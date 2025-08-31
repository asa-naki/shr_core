#include "motor_control_lib/servo_control.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <cerrno>
#include <cstring>
#include <chrono>
#include <thread>
#include <iostream>

namespace motor_control_lib
{

FeetechServoController::FeetechServoController(const std::string& port, int baudrate)
    : port_(port), baudrate_(baudrate), serial_fd_(-1), connected_(false)
{
    initializeRegisterMap();
}

FeetechServoController::~FeetechServoController()
{
    disconnect();
}

bool FeetechServoController::connect()
{
    // シリアルポートを開く
    serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd_ == -1) {
        std::cerr << "Failed to open serial port: " << port_ << " - " << strerror(errno) << std::endl;
        return false;
    }

    // シリアルポート設定
    struct termios options;
    tcgetattr(serial_fd_, &options);

    // ボーレート設定
    speed_t speed;
    switch (baudrate_) {
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        case 460800: speed = B460800; break;
        case 921600: speed = B921600; break;
        default: 
            std::cerr << "Unsupported baudrate: " << baudrate_ << std::endl;
            close(serial_fd_);
            return false;
    }

    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    // 8N1設定
    options.c_cflag &= ~PARENB;  // パリティなし
    options.c_cflag &= ~CSTOPB;  // ストップビット1
    options.c_cflag &= ~CSIZE;   // データビットマスクをクリア
    options.c_cflag |= CS8;      // データビット8
    
    // ハードウェアフロー制御無効
    options.c_cflag &= ~CRTSCTS;
    
    // ローカルライン、受信有効
    options.c_cflag |= CREAD | CLOCAL;
    
    // 入力処理フラグ
    options.c_iflag &= ~(IXON | IXOFF | IXANY);  // ソフトウェアフロー制御無効
    options.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Raw入力
    
    // 出力処理フラグ
    options.c_oflag &= ~OPOST;  // Raw出力
    
    // タイムアウト設定
    options.c_cc[VMIN] = 0;   // 最小読み取り文字数
    options.c_cc[VTIME] = 20; // タイムアウト（0.1秒単位）

    if (tcsetattr(serial_fd_, TCSANOW, &options) != 0) {
        std::cerr << "Failed to set serial attributes: " << strerror(errno) << std::endl;
        close(serial_fd_);
        return false;
    }

    // バッファをクリア
    tcflush(serial_fd_, TCIOFLUSH);

    connected_ = true;
    std::cout << "Connected to servo controller: " << port_ << " @ " << baudrate_ << " bps" << std::endl;
    return true;
}

void FeetechServoController::disconnect()
{
    if (serial_fd_ != -1) {
        close(serial_fd_);
        serial_fd_ = -1;
        connected_ = false;
        std::cout << "Disconnected from servo controller" << std::endl;
    }
}

bool FeetechServoController::isConnected() const
{
    return connected_;
}

uint16_t FeetechServoController::calculateCRC16(const uint8_t* data, size_t length)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

size_t FeetechServoController::createModbusCommand(uint8_t slave_id, uint8_t function_code, uint16_t address, uint16_t value, uint8_t* command)
{
    command[0] = slave_id;
    command[1] = function_code;
    command[2] = (address >> 8) & 0xFF;  // アドレス上位
    command[3] = address & 0xFF;         // アドレス下位
    command[4] = (value >> 8) & 0xFF;    // 値上位
    command[5] = value & 0xFF;           // 値下位
    
    uint16_t crc = calculateCRC16(command, 6);
    command[6] = crc & 0xFF;         // CRC下位
    command[7] = (crc >> 8) & 0xFF;  // CRC上位
    
    return 8;
}

bool FeetechServoController::verifyChecksum(const uint8_t* data, size_t length)
{
    if (length < 3) {
        return false;
    }
    
    // データ部分（最後の2バイトを除く）
    size_t data_length = length - 2;
    uint16_t received_crc = data[length - 2] | (data[length - 1] << 8);
    uint16_t calculated_crc = calculateCRC16(data, data_length);
    
    return received_crc == calculated_crc;
}

int FeetechServoController::sendCommand(const uint8_t* cmd_bytes, size_t cmd_length, uint8_t* response, size_t max_response_length, bool expect_response)
{
    if (!connected_ || serial_fd_ == -1) {
        return -1;
    }

    try {
        // バッファクリア
        tcflush(serial_fd_, TCIOFLUSH);
        
        // RS485送信制御（RTSピン制御）
        int rts = TIOCM_RTS;
        ioctl(serial_fd_, TIOCMBIS, &rts);  // RTS有効
        
        // コマンド送信
        ssize_t bytes_written = write(serial_fd_, cmd_bytes, cmd_length);
        if (bytes_written != static_cast<ssize_t>(cmd_length)) {
            std::cerr << "Failed to write complete command" << std::endl;
            return -1;
        }
        
        // 送信完了待機
        tcdrain(serial_fd_);
        
        // RS485受信制御
        ioctl(serial_fd_, TIOCMBIC, &rts);  // RTS無効
        
        if (!expect_response) {
            return 0;
        }
        
        // 応答受信待機
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        // 応答受信
        ssize_t bytes_read = read(serial_fd_, response, max_response_length);
        if (bytes_read > 0) {
            // チェックサム検証
            if (verifyChecksum(response, bytes_read)) {
                return bytes_read;
            } else {
                std::cerr << "Checksum verification failed" << std::endl;
                return -1;
            }
        } else {
            return -1;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Communication error: " << e.what() << std::endl;
        return -1;
    }
}

int32_t FeetechServoController::readRegister(uint8_t servo_id, uint16_t address)
{
    if (!connected_) {
        return -1;
    }

    uint8_t cmd[8];
    size_t cmd_length = createModbusCommand(servo_id, 3, address, 1, cmd);
    
    uint8_t response[50];
    int response_length = sendCommand(cmd, cmd_length, response, sizeof(response));
    
    if (response_length >= 5 && response[1] == 3) {
        // レジスタ値を抽出（ビッグエンディアン）
        uint16_t value = (response[3] << 8) | response[4];
        return static_cast<int32_t>(value);
    } else if (response_length > 0 && (response[1] & 0x80)) {
        // エラーレスポンス
        return -1;
    } else {
        return -1;
    }
}

bool FeetechServoController::writeRegister(uint8_t servo_id, uint16_t address, uint16_t value)
{
    if (!connected_) {
        return false;
    }
    
    uint8_t cmd[8];
    size_t cmd_length = createModbusCommand(servo_id, 6, address, value, cmd);
    
    uint8_t response[50];
    int response_length = sendCommand(cmd, cmd_length, response, sizeof(response));
    
    if (response_length >= 6) {
        // エコーバック確認
        uint16_t echo_addr = (response[2] << 8) | response[3];
        uint16_t echo_value = (response[4] << 8) | response[5];
        
        return (echo_addr == address && echo_value == value);
    } else {
        return false;
    }
}

bool FeetechServoController::setPosition(uint8_t servo_id, uint16_t position, bool enable_torque, double timeout)
{
    if (!connected_) {
        return false;
    }
    
    try {
        // 1. トルク有効化（必要に応じて）
        if (enable_torque) {
            if (!writeRegister(servo_id, 129, 1)) {  // Torque Enable
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // 2. 位置コマンド送信
        if (!writeRegister(servo_id, 128, position)) {  // Goal Position
            return false;
        }
        
        // 3. タイムアウト処理（将来の拡張用）
        (void)timeout;  // 未使用パラメーター警告を回避
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "setPosition error: " << e.what() << std::endl;
        return false;
    }
}

int32_t FeetechServoController::getCurrentPosition(uint8_t servo_id)
{
    return readRegister(servo_id, 256);  // Present Position
}

void FeetechServoController::initializeRegisterMap()
{
    known_registers_[0] = {"Firmware main version No", 2009, "read", "ファームウェアメインバージョン番号"};
    known_registers_[1] = {"Firmware sub version No", 2005, "read", "ファームウェアサブバージョン番号"};
    known_registers_[2] = {"Firmware Release version No", 2025, "read", "ファームウェアリリースバージョン番号"};
    known_registers_[3] = {"Firmware Release date", 423, "read", "ファームウェアリリース日"};
    known_registers_[10] = {"ID", 10, "read_write", "ID"};
    known_registers_[11] = {"Baudrate", 2, "read_write", "ボーレート"};
    known_registers_[12] = {"Return Delay Time", 500, "read_write", "リターン遅延時間"};
    known_registers_[128] = {"Goal Position", 2129, "read_write", "位置コマンド"};
    known_registers_[129] = {"Torque Enable", 1, "read_write", "トルク有効"};
    known_registers_[130] = {"Goal Acceleration", 0, "read_write", "目標加速度"};
    known_registers_[131] = {"Goal Velocity", 250, "read_write", "目標速度"};
    known_registers_[256] = {"Present Position", 2128, "read", "現在位置"};
    known_registers_[257] = {"Present Position", 2128, "read", "現在位置"};
    known_registers_[258] = {"Present Velocity", 500, "read", "現在速度"};
    known_registers_[259] = {"Present PWM", 500, "read", "現在PWM"};
    known_registers_[260] = {"Present Input Voltage", 500, "read", "電圧フィードバック"};
    known_registers_[261] = {"Present Temperature", 500, "read", "温度フィードバック"};
    known_registers_[262] = {"Moving Status", 500, "read", "移動ステータス"};
    known_registers_[263] = {"Present Current", 0, "read", "現在電流"};
}

// ShotController実装
ShotController::ShotController(std::shared_ptr<ServoControllerBase> servo_controller)
    : servo_controller_(servo_controller)
{
}

bool ShotController::aimAt(uint8_t pan_servo_id, uint8_t tilt_servo_id, uint16_t pan_position, uint16_t tilt_position, double timeout)
{
    if (!servo_controller_ || !servo_controller_->isConnected()) {
        return false;
    }
    
    // パンとチルトサーボを同時に移動
    bool pan_success = servo_controller_->setPosition(pan_servo_id, pan_position, true, timeout);
    bool tilt_success = servo_controller_->setPosition(tilt_servo_id, tilt_position, true, timeout);
    
    return pan_success && tilt_success;
}

bool ShotController::fire(uint8_t trigger_servo_id, uint16_t fire_position, uint16_t return_position, double fire_duration)
{
    if (!servo_controller_ || !servo_controller_->isConnected()) {
        return false;
    }
    
    // 1. 射撃位置に移動
    if (!servo_controller_->setPosition(trigger_servo_id, fire_position)) {
        return false;
    }
    
    // 2. 射撃持続時間待機
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(fire_duration * 1000)));
    
    // 3. 復帰位置に戻る
    return servo_controller_->setPosition(trigger_servo_id, return_position);
}

bool ShotController::returnHome(uint8_t pan_servo_id, uint8_t tilt_servo_id, uint8_t trigger_servo_id, 
                               uint16_t pan_home, uint16_t tilt_home, uint16_t trigger_home)
{
    if (!servo_controller_ || !servo_controller_->isConnected()) {
        return false;
    }
    
    // 全てのサーボをホーム位置に戻す
    bool pan_success = servo_controller_->setPosition(pan_servo_id, pan_home);
    bool tilt_success = servo_controller_->setPosition(tilt_servo_id, tilt_home);
    bool trigger_success = servo_controller_->setPosition(trigger_servo_id, trigger_home);
    
    return pan_success && tilt_success && trigger_success;
}

bool ShotController::getCurrentAim(uint8_t pan_servo_id, uint8_t tilt_servo_id, int32_t& pan_position, int32_t& tilt_position)
{
    if (!servo_controller_ || !servo_controller_->isConnected()) {
        return false;
    }
    
    pan_position = servo_controller_->getCurrentPosition(pan_servo_id);
    tilt_position = servo_controller_->getCurrentPosition(tilt_servo_id);
    
    return (pan_position != -1 && tilt_position != -1);
}

}  // namespace motor_control_lib
