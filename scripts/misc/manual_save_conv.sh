#!/bin/bash
TRANSCRIPT="/mnt/c/Users/USER/.claude/projects/C--Users-USER-Desktop-ARGOS/55109855-69cb-4067-9ff2-eaf107235059.jsonl"
LOG_DIR="/home/jamonnine/.claude/conversation-logs"
TIMESTAMP=$(date +%Y-%m-%d_%H-%M-%S)

mkdir -p "$LOG_DIR"
cp "$TRANSCRIPT" "$LOG_DIR/conversation_${TIMESTAMP}.jsonl"
ln -sf "$LOG_DIR/conversation_${TIMESTAMP}.jsonl" "$LOG_DIR/conversation_latest.jsonl"

echo "✓ Saved: $LOG_DIR/conversation_${TIMESTAMP}.jsonl"
ls -la "$LOG_DIR/" | tail -5
