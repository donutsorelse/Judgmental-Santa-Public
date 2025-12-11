# Judgmental Santa 🎅

I built an AI-powered Santa that judges whether you're "Naughty" or "Nice" and either wags his finger at you or throws candy at you with a robot arm. Watch it in action here:

**📹 [Watch the full video on YouTube!](https://www.youtube.com/watch?v=jUSmzbQnJgo)**

## What It Does

Santa uses a webcam and OpenAI's vision AI to look at you, decides if you're naughty or nice (kids are always nice, adults... not so much), then speaks his judgment out loud. If you're nice, a 6-axis robot arm throws candy at you. If you're naughty, he wags his finger. An ultrasonic sensor detects when someone walks up so it runs automatically.

## Hardware I Used

- Raspberry Pi 4
- USB webcam
- USB speakers
- HC-SR04 ultrasonic sensor (detects motion)
- 2 servo motors for the finger wagging
- LeRobot arm with 6 Feetech STS3215 servos for candy throwing

## Setup

You'll need an OpenAI API key. Either set it as an environment variable or just hardcode it in the script on line 24.

```bash
export OPENAI_API_KEY="sk-your-key-here"
```

Install the dependencies:
```bash
pip install openai opencv-python pigpio playsound lerobot
sudo apt-get install mpg123 alsa-utils
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
```

Then just run it:
```bash
python3 judgmental_santa.py
```

If you want it to auto-start on boot, there's a systemd service example in the full README below.

## GPIO Pins

- Ultrasonic sensor: GPIO 23 (trigger), GPIO 24 (echo)
- Servos: GPIO 10 (shoulder), GPIO 18 (hand)
- Robot arm: USB serial (/dev/ttyACM0)

## Notes

The script expects a `sequences/throw_candy_at_kids.json` file with the robot arm positions. It's included in this repo.

Audio tries USB first (plughw:4,0) then falls back to onboard audio. If you have different audio hardware, you might need to tweak the device names in the script.

The robot arm setup was the hardest part - had to figure out that the motors needed `Lock=1` and higher torque settings to actually lift against gravity. Positions 10 and 11 kept failing until I manually recorded the correct positions and cranked up the P and D coefficients.

---

