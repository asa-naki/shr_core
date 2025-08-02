# !/usr/bin/python3
# -*- coding: utf-8 -*-

import time
from gpiozero import Servo
import sys
import platform

# システム情報を表示
print(f"🔍 システム情報: {platform.system()} {platform.machine()}")
try:
    with open('/proc/cpuinfo', 'r') as f:
        for line in f:
            if 'Model' in line:
                print(f"🔍 {line.strip()}")
                break
except:
    pass

# pigpioピンファクトリーを使用を試行、失敗した場合はソフトウェアPWMを使用
pin_factory_used = "デフォルト"
try:
    from gpiozero.pins.pigpio import PiGPIOFactory
    from gpiozero import Device
    
    # pigpiodが動作しているかテスト
    test_factory = PiGPIOFactory()
    Device.pin_factory = test_factory
    pin_factory_used = "pigpio（高精度PWM）"
    print(f"✅ {pin_factory_used}を使用します")

except ImportError:
    print("⚠️ pigpioライブラリがインストールされていません。デフォルトPWMを使用します")
except Exception as e:
    print(f"⚠️ pigpioが利用できません（理由: {str(e)[:50]}...）")
    print("デフォルトPWMを使用します")
    # デフォルトピンファクトリーに戻す
    try:
        from gpiozero import Device
        Device.pin_factory = None
    except:
        pass

PIN = 13
# ESC用のパルス幅設定（1.0ms〜2.0msがより安全）
# 多くのESCは1.5msが中立点、1.0msが逆転最大、2.0msが前進最大

print("=== ⚠️  安全確認 ⚠️  ===")
print("🚨 重要: 初期化中にモーターが回転する可能性があります！")
print("以下の安全対策を必ず実施してください：")
print("1. プロペラを取り外してください")
print("2. または、モーターを安全に固定してください")
print("3. 人や物がモーター/プロペラから離れていることを確認してください")
print("4. バッテリーを外してください")
print("5. ESCの電源も切ってください")
print()
confirm = input("安全対策を実施しましたか？ (yes/y で続行): ").strip().lower()
if confirm not in ['yes', 'y']:
    print("安全対策を実施してから再実行してください。")
    exit()

print("\n=== ESC初期化プロセス ===")
print("⚠️  注意: 初期化中はモーターに触れないでください")

# Servoオブジェクトを安全に作成
try:
    servo = Servo(PIN, min_pulse_width=1.0/1000, max_pulse_width=2.0/1000)
    print(f"✅ GPIO{PIN}でServoオブジェクトを作成しました（{pin_factory_used}使用）")
except Exception as e:
    error_msg = str(e).lower()
    print(f"❌ Servoオブジェクトの作成に失敗しました: {e}")
    print("\n🔧 トラブルシューティング:")
    
    if "permission" in error_msg or "cannot determine soc" in error_msg:
        print("🚨 GPIO権限の問題が検出されました！")
        print("📋 解決方法（以下を順番に試してください）:")
        print("1. sudo権限で実行: sudo python3 test_esc.py")
        print("2. ログアウト/ログインして権限を更新")
        print("3. システム再起動: sudo reboot")
        print("4. 権限設定スクリプトを実行:")
        print("   sudo usermod -a -G gpio,dialout $USER")
        print("   sudo chmod 666 /dev/gpiomem*")
    else:
        print("1. Raspberry Piで実行していますか？")
        print("2. sudo権限で実行してみてください: sudo python3 test_esc.py") 
        print("3. 他のプロセスがGPIOを使用していないか確認してください")
        print("4. システムを再起動してみてください")
    
    sys.exit(1)

# 初期状態を停止に設定
try:
    servo.min()  # 停止状態
    print("✅ 初期状態を停止に設定しました")
except Exception as e:
    print(f"⚠️ 初期設定中にエラー: {e}")
    print("継続して実行しますが、動作に問題がある場合は再起動してください")
# ESC初期化シーケンス：最大スロットル位置に設定



print("=== ESC初期化完了 ===")
print("✅ ESCが初期化されました。停止状態で待機中...")
print("⚠️  プロペラを取り付ける前に動作テストを行ってください")
print()
print("=== 制御コマンド ===")
print("\"max\" または \"m\" - 最大回転")
print("\"stop\" または \"s\" - 停止")
print("\"zero\" または \"z\" - PWM信号停止")
print("\"quit\" または \"q\" - 終了")
print("Ctrl+C - 緊急停止")

# 初期状態：停止
is_running = False
print()
print("🔴 現在の状態: 停止")
print("プロペラなしで動作確認してから使用してください")


def test_pattern():
    """テストパターン実行"""
    print("=== テストパターン実行 ===")
    test_speeds = [0.1, 0.2, 0.3, 0.4, 0.5]
    
    try:
        for speed_val in test_speeds:
            print(f"テスト速度: {speed_val:.1f}")
            servo.value = speed_val
            time.sleep(2)
            
            print("停止")
            servo.value = 0
            time.sleep(1)
    
    except KeyboardInterrupt:
        print("\nテストパターンが中断されました")
        servo.value = 0
    
    print("=== テストパターン終了 ===")
    servo.value = 0

try:
    # 初期状態は停止
    servo.min()  # 停止状態
    
    while True:    
        status = "🟢 最大回転中" if is_running else "🔴 停止中"
        print(f"\n現在の状態: {status}")
        inp = input("コマンド入力: ").strip().lower()
        
        if inp == "max" or inp == "m":
            if not is_running:
                print("⚠️  最大回転を開始します！")
                confirm = input("本当に実行しますか？ (yes/y で実行): ").strip().lower()
                if confirm in ['yes', 'y']:
                    print("🟢 最大回転開始！")
                    servo.max()  # 最大回転
                    is_running = True
                else:
                    print("キャンセルしました")
            else:
                print("既に最大回転中です")
                
        elif inp == "stop" or inp == "s":
            if is_running:
                print("🔴 停止します")
                servo.min()  # 停止
                is_running = False
            else:
                print("既に停止中です")
                
        elif inp == "zero" or inp == "z":
            print("PWM信号を停止します")
            servo.close()
            print("パルス幅を0にしました（PWM信号停止）")
            print("プログラムを終了します")
            break
            
        elif inp == "quit" or inp == "q":
            break
            
        else:
            print("使用可能コマンド: max(最大回転), stop(停止), zero(PWM停止), quit(終了)")

except KeyboardInterrupt:
    print("\nCtrl+Cが押されました。安全に終了します。")
    servo.min()  # 停止状態に設定
    time.sleep(0.5)

finally:
    # 終了処理
    print("終了処理を実行中...")
    try:
        servo.min()  # 停止状態に設定してから終了
        time.sleep(0.5)
        servo.close()
    except:
        pass
    print("プログラムを終了しました。")
