#!/usr/bin/env python3
import os
import random
import sys

def generate_math_problem():
    """生成一个简单的四则运算问题并返回播报文本"""
    a = random.randint(1, 10)
    b = random.randint(1, 10)
    op = random.choice(['+', '-', '*'])
    
    if op == '+':
        return f"{a} 加 {b} 等于 {a + b}"
    elif op == '-':
        if a < b:
            a, b = b, a # 保证结果为正数，方便听取
        return f"{a} 减 {b} 等于 {a - b}"
    elif op == '*':
        return f"{a} 乘以 {b} 等于 {a * b}"

def speak(text):
    """尝试使用不同的引擎来播报文本"""
    print(f"准备播报内容: 【{text}】")
    
    # 尝试 1: 使用 espeak (Linux 常用命令行 TTS 工具)
    print("-> 尝试使用 espeak 播报 (已调至最大音量)...")
    # -v zh 代表中文, -a 200 代表将音量放大到 200 (默认 100)
    if os.system(f'espeak -v zh -a 200 "{text}" 2>/dev/null') == 0:
        return True
        
    # 尝试 2: 使用 pyttsx3
    print("-> espeak 失败或未安装，尝试使用 pyttsx3...")
    try:
        import pyttsx3
        engine = pyttsx3.init()
        engine.say(text)
        engine.runAndWait()
        return True
    except ImportError:
        print("   pyttsx3 未安装。")
    except Exception as e:
        print(f"   pyttsx3 运行失败: {e}")
        
    # 尝试 3: 使用 gTTS 和 mpg123
    print("-> 尝试使用 gTTS 播报...")
    try:
        from gtts import gTTS
        tts = gTTS(text=text, lang='zh-cn')
        audio_file = "/tmp/test_math_audio.mp3"
        tts.save(audio_file)
        if os.system(f"mpg123 {audio_file} 2>/dev/null") == 0 or os.system(f"mplayer {audio_file} 2>/dev/null") == 0:
            return True
    except ImportError:
        print("   gTTS 未安装。")
    except Exception as e:
        print(f"   gTTS 运行失败: {e}")
    
    print("\n[!] 无法播报。请安装以下任意一种语音依赖以支持发声：")
    print("  1. 系统级工具: sudo apt-get install espeak")
    print("  2. Python 库: pip install pyttsx3")
    print("  3. Python 库 + 播放器: pip install gTTS && sudo apt-get install mpg123")
    return False

if __name__ == "__main__":
    print("="*30)
    print("      喇叭与四则运算播报测试")
    print("="*30)
    
    text = generate_math_problem()
    success = speak(text)
    
    if success:
        print("\n=> 播报测试完成！如果喇叭好使，你应该已经听到了声音。")
    else:
        print("\n=> 播报测试失败，由于缺少 TTS 依赖，请按提示安装后再试。")
        sys.exit(1)
