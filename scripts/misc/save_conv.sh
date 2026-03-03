#!/bin/bash
# Fix CRLF and save conversation
SCRIPT="/mnt/c/Users/USER/.claude/skills/conversation-logger/scripts/save-conversation.sh"
PARSER="/mnt/c/Users/USER/.claude/skills/conversation-logger/scripts/parse-conversation.py"
TRANSCRIPT="/mnt/c/Users/USER/.claude/projects/C--Users-USER-Desktop-ARGOS/55109855-69cb-4067-9ff2-eaf107235059.jsonl"

# Convert CRLF to LF
tr -d '\r' < "$SCRIPT" > /tmp/save-conversation.sh
tr -d '\r' < "$PARSER" > /tmp/parse-conversation.py
chmod +x /tmp/save-conversation.sh

# Run the save script
bash /tmp/save-conversation.sh "$TRANSCRIPT" "55109855-day15-nav2" 2>&1
