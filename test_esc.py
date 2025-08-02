# !/usr/bin/python3
# -*- coding: utf-8 -*-

import time
from gpiozero import Servo
import sys

# pigpioピンファクトリーを使用を試行、失敗した場合はソフトウェアPWMを使用
try:
    from gpiozero.pins.pigpio import PiGPIOFactory
    from gpiozero import Device
    Device.pin_factory = PiGPIOFactory()
    print("pigpioピンファクトリーを使用します（高精度PWM）")
except Exception as e:
    print(f"pigpioが利用できません。ソフトウェアPWMを使用します: {e}")

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
servo = Servo(PIN, min_pulse_width=1.0/1000, max_pulse_width=2.0/1000)
servo.min()
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
