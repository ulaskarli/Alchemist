#!/bin/bash
python main.py &
python3.8 LLM/chatgpt.py &
# python3.8 LLM/bard.py &
python3.8 LLM/whisper_ros.py --model "base" &

