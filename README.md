# AIBell
AI doorbell with DFRobot ESP32S3 AI Camera Module, Edge Impulse and chatGPT

# Parts required
AI Camera module 1.0 DFR1154
microSD card

# Machine Learning platform
Edge Impulse

# Setup
Install Universal Telegram Bot library
Get an OpenAI API key (for Whisper transcription and LLM answers) at https://platform.openai.com/settings/organization/api-keys
Get a Telegram bot token (for sending notifications) https://core.telegram.org/bots/tutorial 
Edit credentials and settings inside the .ino code

threshold=0.7; // for face recognition
const char* ssid = ""; // WiFi Credentials
const char* password = "";
const char* openai_api_key = "sk-proj….";  // OpenAI API key

Upload the code, selecting ESP32S3 Dev Module as the board and configuring USB CDC On Boot, Partition 16mb Flash 3mb 9.9, Flash 16 128, OPI PSRAM.

# Enclosure 
Download and print https://cults3d.com/en/3d-model/gadget/aibell

# Contact
Roni Bandini
@ronibandini
