import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import numpy as np
import sounddevice as sd
import matplotlib.pyplot as plt

# pip install sounddevice
# sudo apt-get install portaudio19-dev

class AudioAnalyzerNode(Node):
    def __init__(self):
        super().__init__('audio_analyzer')
        self.publisher_ = self.create_publisher(Bool, '/click_loc', 10)

        # Sounddevice settings
        self.RATE = 44100
        self.BUFFER_SIZE = 16384

        self.min_freq = 2000
        self.max_freq = 4000
        self.mag_threshold = 1000000

    def run(self):
        data_buffer = np.zeros(self.BUFFER_SIZE*16, int)

        fig = plt.figure()
        fft_fig = fig.add_subplot(2, 1, 1)
        wave_fig = fig.add_subplot(2, 1, 2)

        while rclpy.ok():
            try:
                # Read audio data from stream
                audio_data = sd.rec(self.BUFFER_SIZE, samplerate=self.RATE, channels=1, dtype='int16')
                sd.wait()

                fft_data = np.fft.fft(audio_data[:, 0])
                freq = np.fft.fftfreq(self.BUFFER_SIZE, d=1/self.RATE)

                # Check if any magnitude in the specified frequency range exceeds the threshold
                freq_range_indices = np.where((freq >= self.min_freq) & (freq <= self.max_freq ))[0]
                magnitudes_in_range = np.abs(fft_data[freq_range_indices])
                if any(magnitudes_in_range > self.mag_threshold ):
                    msg = Bool()
                    msg.data = True
                    self.publisher_.publish(msg)

                # plot
                data_buffer = np.append(data_buffer[self.BUFFER_SIZE:], audio_data[:, 0])
                wave_fig.clear()
                fft_fig.clear()
                wave_fig.plot(data_buffer)
                fft_fig.plot(freq[:self.BUFFER_SIZE//4], np.abs(fft_data[:self.BUFFER_SIZE//4]))
                wave_fig.set_ylim(-10000, 10000)

                plt.pause(0.001)

            except KeyboardInterrupt:
                break

    def cleanup(self):
        self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()

def main(args=None):
    rclpy.init(args=args)
    node = AudioAnalyzerNode()
    try:
        node.run()
    finally:
        node.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()