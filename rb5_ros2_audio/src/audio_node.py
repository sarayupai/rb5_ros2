#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyaudio
import whisper
import wave

class AudioNode(Node):
    def __init__(self):
        super().__init__('audio_module')
        self.publisher = self.create_publisher(String, '/transcribed_speech', 10)
        self.subscription = self.create_subscription(String, '/state', self.update_state, 10)
        self.model = whisper.load_model("base")
        print("Model loaded")
        self.timer = self.create_timer(5.0, self.record_and_transcribe)
        self.recording_enabled = False 

    def update_state(self, msg):
        if msg.data == 'record':
            self.recording_enabled = True
        else:
            self.recording_enabled = False 
        self.get_logger().info(f"[RECORD STATE]: {self.recording_enabled}")
        
    # Record mic audio, transcribe using Whisper, and publish text every 5 secs if recording is enabled 
    def record_and_transcribe(self):
        if self.recording_enabled:
            # Capture audio from the microphone
            audio_file = self.capture_audio()

            # Transcribe audio using Whisper
            text = self.transcribe_audio(audio_file)

            # Publish the recognized text to a topic
            msg = String()
            msg.data = text
            self.publisher.publish(msg)

    # Capture mic audio using pyaudio 
    def capture_audio(self):
        p = pyaudio.PyAudio()
        stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=1024)
        frames = []

        print("Recording...")
        for _ in range(0, int(16000 / 1024 * 5)):  # Record for 5 seconds
            data = stream.read(1024)
            frames.append(data)

        print("Finished recording.")
        stream.stop_stream()
        stream.close()
        p.terminate()

        # Save the audio to a temporary WAV file
        with wave.open("temp_audio.wav", 'wb') as wf:
            wf.setnchannels(1)
            wf.setsampwidth(p.get_sample_size(pyaudio.paInt16))
            wf.setframerate(16000)
            wf.writeframes(b''.join(frames))
        return "temp_audio.wav"

    # Transcribe audio using Whisper
    def transcribe_audio(self, audio_file):
        print("Transcribing ...")
        result = self.model.transcribe(audio_file)
        print(result["text"])
        return result["text"]

def main(args=None):
    print('here')
    rclpy.init(args=args)
    node = AudioNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()