import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import numpy as np
import time
import sounddevice as sd

class AudioAnalyzerNode(Node):
    def __init__(self):
        super().__init__('audio_analyzer')
        self.publisher_ = self.create_publisher(Bool, '/click_loc', 10)

        # Sounddevice settings
        self.RATE = 44100
        self.BLOCK_SIZE = 3000
        self.MIN_FREQ = 2000
        self.MAX_FREQ = 4000
        self.MAG_THRESHOLD = 1000000

        # Setup audio stream
        self.audio_stream = sd.InputStream(
            samplerate=self.RATE,
            blocksize=self.BLOCK_SIZE,
            channels=1,
            dtype='int16',
            callback=self.audio_callback
        )

    def run(self):
        with self.audio_stream:
            rclpy.spin(self)

    def audio_callback(self, indata, frames, time, status):
        if status:
            print(status)
        else:
            fft_data = np.fft.fft(indata[:, 0], n=self.BLOCK_SIZE)
            freq = np.fft.fftfreq(self.BLOCK_SIZE, d=1/self.RATE)

            # Check if any magnitude in the specified frequency range exceeds the threshold
            freq_range_indices = np.where((freq >= self.MIN_FREQ) & (freq <= self.MAX_FREQ))[0]
            magnitudes_in_range = np.abs(fft_data[freq_range_indices])
            if any(magnitudes_in_range > self.MAG_THRESHOLD):
                msg = Bool()
                msg.data = True
                self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AudioAnalyzerNode()
    try:
        node.run()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()