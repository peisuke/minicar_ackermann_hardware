#!/usr/bin/env python3
"""
PCA9685直接制御によるサーボテストスクリプト

ROS2なしで動作。ハードウェア接続の確認用。

Usage:
  sudo python3 servo_test.py

Requirements:
  pip install smbus2
"""

import time

try:
    import smbus2
except ImportError:
    print("smbus2が必要です: pip install smbus2")
    exit(1)


# =============================================================================
# PCA9685 設定
# =============================================================================
PCA9685_ADDR = 0x40      # I2Cアドレス
I2C_BUS = 1              # Raspberry Pi のI2Cバス

# レジスタ
MODE1 = 0x00
PRESCALE = 0xFE
LED0_ON_L = 0x06

# PWM設定
PWM_FREQ = 50            # 50Hz (サーボ標準)
OSC_CLOCK = 25000000     # 内部クロック 25MHz


# =============================================================================
# チャンネル割り当て
# =============================================================================
ESC_CHANNEL = 0          # ESC (スロットル)
SERVO_CHANNEL = 1        # ステアリングサーボ


# =============================================================================
# サーボ設定 (パルス幅 ms)
# =============================================================================
SERVO_CENTER_MS = 1.5    # 中央
SERVO_LEFT_MS = 1.0      # 左最大
SERVO_RIGHT_MS = 2.0     # 右最大


class PCA9685:
    """PCA9685 PWMコントローラ"""

    def __init__(self, address=PCA9685_ADDR, bus_num=I2C_BUS):
        self.address = address
        self.bus = smbus2.SMBus(bus_num)
        self._init_pca9685()

    def _init_pca9685(self):
        """PCA9685を初期化して50Hzに設定"""
        # スリープモードに入る
        self.bus.write_byte_data(self.address, MODE1, 0x10)
        time.sleep(0.005)

        # プリスケーラ設定 (50Hz)
        # prescale = round(OSC_CLOCK / (4096 * freq)) - 1
        prescale = round(OSC_CLOCK / (4096 * PWM_FREQ)) - 1
        self.bus.write_byte_data(self.address, PRESCALE, prescale)

        # スリープ解除、自動インクリメント有効
        self.bus.write_byte_data(self.address, MODE1, 0x20)
        time.sleep(0.005)

        print(f"PCA9685初期化完了 (アドレス: 0x{self.address:02X}, 周波数: {PWM_FREQ}Hz)")

    def set_pwm(self, channel, on, off):
        """指定チャンネルのPWM設定"""
        reg = LED0_ON_L + 4 * channel
        self.bus.write_byte_data(self.address, reg, on & 0xFF)
        self.bus.write_byte_data(self.address, reg + 1, on >> 8)
        self.bus.write_byte_data(self.address, reg + 2, off & 0xFF)
        self.bus.write_byte_data(self.address, reg + 3, off >> 8)

    def set_pulse_ms(self, channel, pulse_ms):
        """パルス幅(ms)でPWM設定"""
        # 50Hz = 20ms周期, 4096ステップ
        # pulse_ms / 20ms * 4096 = PWM値
        pwm_value = int(pulse_ms / 20.0 * 4096)
        pwm_value = max(0, min(4095, pwm_value))
        self.set_pwm(channel, 0, pwm_value)

    def close(self):
        """I2Cバスを閉じる"""
        self.bus.close()


def test_servo():
    """サーボテストシーケンス"""
    print("=" * 50)
    print("サーボテスト開始")
    print("=" * 50)
    print()

    try:
        pwm = PCA9685()
    except Exception as e:
        print(f"エラー: PCA9685に接続できません")
        print(f"  {e}")
        print()
        print("確認事項:")
        print("  1. I2Cが有効か: sudo raspi-config → Interface Options → I2C")
        print("  2. 接続確認: sudo i2cdetect -y 1")
        print("  3. root権限で実行: sudo python3 servo_test.py")
        return

    try:
        # 中央位置
        print(f"1. 中央位置 ({SERVO_CENTER_MS}ms)")
        pwm.set_pulse_ms(SERVO_CHANNEL, SERVO_CENTER_MS)
        time.sleep(2)

        # 左
        print(f"2. 左 ({SERVO_LEFT_MS}ms)")
        pwm.set_pulse_ms(SERVO_CHANNEL, SERVO_LEFT_MS)
        time.sleep(2)

        # 右
        print(f"3. 右 ({SERVO_RIGHT_MS}ms)")
        pwm.set_pulse_ms(SERVO_CHANNEL, SERVO_RIGHT_MS)
        time.sleep(2)

        # 中央に戻る
        print(f"4. 中央に戻る ({SERVO_CENTER_MS}ms)")
        pwm.set_pulse_ms(SERVO_CHANNEL, SERVO_CENTER_MS)
        time.sleep(1)

        print()
        print("テスト完了")

    except KeyboardInterrupt:
        print("\n中断されました")
    finally:
        # 中央に戻してから終了
        pwm.set_pulse_ms(SERVO_CHANNEL, SERVO_CENTER_MS)
        pwm.close()


def test_esc():
    """ESCテスト (ニュートラルのみ)"""
    print("=" * 50)
    print("ESC ニュートラル設定")
    print("=" * 50)
    print()
    print("注意: ESCのキャリブレーションが必要な場合があります")
    print()

    try:
        pwm = PCA9685()
    except Exception as e:
        print(f"エラー: {e}")
        return

    try:
        # ニュートラル (1.5ms)
        print("ニュートラル信号を送信中 (1.5ms)...")
        print("ESCの電源を入れてキャリブレーションしてください")

        for i in range(30):  # 30秒間送信
            pwm.set_pulse_ms(ESC_CHANNEL, 1.5)
            time.sleep(1)
            print(f"  {30-i}秒...")

    except KeyboardInterrupt:
        print("\n中断されました")
    finally:
        pwm.set_pulse_ms(ESC_CHANNEL, 1.5)
        pwm.close()


def interactive_servo():
    """対話的サーボ制御"""
    print("=" * 50)
    print("対話的サーボ制御")
    print("=" * 50)
    print()
    print("コマンド:")
    print("  数値 (0.5-2.5): パルス幅を直接指定 (ms)")
    print("  l: 左 (1.0ms)")
    print("  c: 中央 (1.5ms)")
    print("  r: 右 (2.0ms)")
    print("  q: 終了")
    print()

    try:
        pwm = PCA9685()
    except Exception as e:
        print(f"エラー: {e}")
        return

    try:
        while True:
            cmd = input("コマンド> ").strip().lower()

            if cmd == 'q':
                break
            elif cmd == 'l':
                pulse = SERVO_LEFT_MS
            elif cmd == 'c':
                pulse = SERVO_CENTER_MS
            elif cmd == 'r':
                pulse = SERVO_RIGHT_MS
            else:
                try:
                    pulse = float(cmd)
                    if pulse < 0.5 or pulse > 2.5:
                        print("範囲外です (0.5-2.5)")
                        continue
                except ValueError:
                    print("無効なコマンド")
                    continue

            print(f"  → {pulse}ms")
            pwm.set_pulse_ms(SERVO_CHANNEL, pulse)

    except KeyboardInterrupt:
        print("\n中断されました")
    finally:
        pwm.set_pulse_ms(SERVO_CHANNEL, SERVO_CENTER_MS)
        pwm.close()
        print("サーボを中央に戻しました")


def interactive_esc():
    """対話的ESC制御"""
    print("=" * 50)
    print("対話的ESC制御 (CH0)")
    print("=" * 50)
    print()
    print("【重要】最初に1.5msでESCをアームしてください")
    print()
    print("コマンド:")
    print("  数値 (1.0-2.0): パルス幅を直接指定 (ms)")
    print("  n: ニュートラル (1.5ms)")
    print("  +: 少し前進 (+0.05ms)")
    print("  -: 少し後退 (-0.05ms)")
    print("  s: 停止 (ニュートラルに戻す)")
    print("  q: 終了")
    print()

    try:
        pwm = PCA9685()
    except Exception as e:
        print(f"エラー: {e}")
        return

    current_pulse = 1.5
    print(f"初期値: {current_pulse}ms (ニュートラル)")
    pwm.set_pulse_ms(ESC_CHANNEL, current_pulse)

    try:
        while True:
            cmd = input(f"[{current_pulse:.2f}ms] コマンド> ").strip().lower()

            if cmd == 'q':
                break
            elif cmd == 'n' or cmd == 's':
                current_pulse = 1.5
            elif cmd == '+':
                current_pulse = min(2.0, current_pulse + 0.05)
            elif cmd == '-':
                current_pulse = max(1.0, current_pulse - 0.05)
            else:
                try:
                    pulse = float(cmd)
                    if pulse < 1.0 or pulse > 2.0:
                        print("安全のため 1.0-2.0ms の範囲で入力してください")
                        continue
                    current_pulse = pulse
                except ValueError:
                    print("無効なコマンド")
                    continue

            print(f"  → {current_pulse:.2f}ms")
            pwm.set_pulse_ms(ESC_CHANNEL, current_pulse)

    except KeyboardInterrupt:
        print("\n中断されました")
    finally:
        pwm.set_pulse_ms(ESC_CHANNEL, 1.5)
        pwm.close()
        print("ESCをニュートラルに戻しました")


if __name__ == '__main__':
    import sys

    print()
    print("PCA9685 サーボ/ESC テストツール")
    print()

    if len(sys.argv) > 1:
        mode = sys.argv[1]
    else:
        print("モード選択:")
        print("  1. サーボテスト (自動)")
        print("  2. サーボ制御 (対話)")
        print("  3. ESCニュートラル")
        print("  4. ESC制御 (対話) ★おすすめ")
        print()
        mode = input("選択 (1-4)> ").strip()

    if mode in ['1', 'servo']:
        test_servo()
    elif mode in ['2', 'interactive', 'i']:
        interactive_servo()
    elif mode in ['3', 'esc']:
        test_esc()
    elif mode in ['4', 'esc-i']:
        interactive_esc()
    else:
        print("1, 2, 3, 4 のいずれかを入力してください")
