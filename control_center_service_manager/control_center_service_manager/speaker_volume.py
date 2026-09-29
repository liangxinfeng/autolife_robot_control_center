"""PCH analog output gain, readback, and a short volume preview sound."""
import argparse
import io
import json
import math
import os
from pathlib import Path
import re
import struct
import subprocess
import wave


def mixer(card, *args):
    return subprocess.run(['amixer','-M','-c',str(card),*args],check=True,
        capture_output=True,text=True,timeout=4,env={**os.environ,'LC_ALL':'C'}).stdout


def parse_control(text):
    values=[int(v) for v in re.findall(r'\[(\d+)%\]',text)]
    if not values: raise RuntimeError('声卡没有返回有效的音量反馈')
    return {'percent':min(values),'enabled':'[off]' not in text}


def read(card):
    master=parse_control(mixer(card,'sget','Master'))
    pcm=parse_control(mixer(card,'sget','PCM'))
    return {'card':card,'master':master,'pcm':pcm,
            'volume':master['percent'] if master['enabled'] and pcm['enabled'] and pcm['percent'] else 0}


def apply(card, level):
    if isinstance(level,bool) or not 0<=level<=100:raise ValueError('音量范围为0～100')
    before=read(card)
    try:
        # Hold output muted while restoring the PCM path: never transiently
        # expose full gain if a different application is already speaking.
        mixer(card,'sset','Master','mute')
        mixer(card,'sset','PCM','100%')
        mixer(card,'sset','Master',f'{level}%','mute' if level==0 else 'unmute')
        actual=read(card)
        if (abs(actual['volume']-level)>2 or actual['pcm']['percent']<99
                or (level>0 and not actual['master']['enabled'])):
            raise RuntimeError('扬声器音量写入后读回不一致')
        return actual
    except Exception:
        # Best effort rollback leaves the previous mute state/gain intact.
        try:
            mixer(card,'sset','Master','mute')
            mixer(card,'sset','PCM',f"{before['pcm']['percent']}%")
            mixer(card,'sset','Master',f"{before['master']['percent']}%",
                  'unmute' if before['master']['enabled'] else 'mute')
        except Exception: pass
        raise


def preview(card):
    """Play a brief, soft chime through the same PCH analog output."""
    sample_rate = 48000
    sample_count = int(sample_rate * 0.22)
    samples = bytearray()
    for index in range(sample_count):
        position = index / sample_count
        envelope = min(1.0, position / 0.08) * (1.0 - position) ** 2
        value = int(2147483647 * 0.18 * envelope * math.sin(2 * math.pi * 660 * index / sample_rate))
        samples.extend(struct.pack('<ii', value, value))
    output = io.BytesIO()
    with wave.open(output, 'wb') as sound:
        sound.setnchannels(2)
        sound.setsampwidth(4)
        sound.setframerate(sample_rate)
        sound.writeframes(samples)
    try:
        subprocess.run(
            ['aplay', '-q', '-D', 'dmix:CARD=PCH,DEV=0', '-t', 'wav', '-'],
            input=output.getvalue(), capture_output=True, check=True, timeout=3,
        )
    except subprocess.CalledProcessError as error:
        detail = (error.stderr or b'').decode('utf-8', errors='replace').strip()
        raise RuntimeError(detail or '扬声器试听播放失败') from error


SAVED_VOLUME_PATH = Path('/home/ubuntu/miniconda3/envs/robot_env/lib/python3.12/site-packages/'
                        'autolife_robot_vision/configs/robot_v2_2.json')

def ensure_saved_volume():
    """Share the control-center setting, including intentional mute."""
    level = json.loads(SAVED_VOLUME_PATH.read_text())['audio']['speaker_volume']
    if isinstance(level, bool) or not isinstance(level, (int, float)) or not 0 <= level <= 100:
        raise ValueError('控制中心保存的音量无效')
    cards = re.findall(r'^\s*(\d+)\s+\[\s*PCH\s*\]\s*:', Path('/proc/asound/cards').read_text(), re.M)
    if len(cards) != 1:raise RuntimeError('未找到唯一的PCH扬声器声卡')
    card = int(cards[0]);actual = read(card)
    if abs(actual['volume']-level)>2 or actual['pcm']['percent']<99:
        actual = apply(card, int(level))
    return actual


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--card',type=int)
    parser.add_argument('--volume',type=int)
    parser.add_argument('--read',action='store_true')
    parser.add_argument('--preview',action='store_true')
    args=parser.parse_args()
    try:
        cards=Path('/proc/asound/cards').read_text()
        found=re.findall(r'^\s*(\d+)\s+\[\s*PCH\s*\]\s*:',cards,re.M)
        if len(found)!=1:raise RuntimeError('未找到唯一的PCH扬声器声卡')
        card=int(found[0])
        if args.card is not None and args.card!=card:raise RuntimeError('扬声器声卡编号已变化，请刷新重试')
        if args.read:result=read(card)
        elif args.volume is not None:result=apply(card,args.volume)
        elif args.preview:
            preview(card)
            result={'card':card,'preview':True}
        else:raise ValueError('需要指定读取或设置音量')
        print(json.dumps({'success':True,**result}))
    except Exception as error:
        print(json.dumps({'success':False,'message':str(error)},ensure_ascii=False))
        raise SystemExit(1)


if __name__=='__main__':main()
