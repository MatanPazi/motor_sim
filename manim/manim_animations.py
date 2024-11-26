import os
import platform
from manim import *

from manim import *

class SineWaveWithImpedance(Scene):
    def construct(self):
        # Axes for the sine wave
        axes = Axes(
            x_range=[0, 5, 1],
            y_range=[-1.5, 1.5, 0.5],
            axis_config={"include_numbers": False},
        )
        axes_labels = axes.get_axis_labels(x_label="x", y_label="y")
        
        # Initial sine wave plot
        init_freq = 0.2
        sine_wave = axes.plot(lambda x: np.sin(2 * PI * init_freq * x), x_range=[0, 5], color=BLUE)
        
        # Text for "Impedance"
        impedance_text = Text("Impedance").to_edge(UP)

        # Initialize size_tracker with the text's initial height
        size_tracker = ValueTracker(impedance_text.height)        

        # Add the components to the scene
        self.play(Create(axes), Write(axes_labels))
        self.play(Create(sine_wave), FadeIn(impedance_text))

        # Animation variable
        freq_tracker = ValueTracker(init_freq)  # Start frequency

        # Updating sine wave as frequency increases
        def update_sine_wave(sine):
            sine.become(
                axes.plot(
                    lambda x: np.sin(2 * PI * freq_tracker.get_value() * x),
                    x_range=[0, 5],
                    color=BLUE,
                )
            )
        
        sine_wave.add_updater(update_sine_wave)

        # Updating impedance text as it grows
        def update_impedance_text(text):
            text.set_height(size_tracker.get_value())
        
        impedance_text.add_updater(update_impedance_text)

        # Animate the changes
        self.play(
            freq_tracker.animate.set_value(init_freq * 5),  # Increase frequency
            size_tracker.animate.set_value(impedance_text.height * 1.5),  # Grow text
            run_time=5,
            rate_func=linear,
        )

        # Remove updaters after animation
        sine_wave.remove_updater(update_sine_wave)
        impedance_text.remove_updater(update_impedance_text)
        
        self.wait(2)















if __name__ == "__main__":
    from manim import config

    # Optional: Set output quality here (low or high)
    config.quality = "low_quality"    # Change to "high_quality" for higher resolution
    config.media_dir = os.getcwd()    # Optional: Set output directory

    # Render the scene
    SineWaveWithImpedance().render()

    # Automatically open the output file
    if platform.system() == 'Windows':
        os.system(f"start {config.output_file}")
        input("Press Enter to exit...")  # Prevents the script from closing immediately. Need in Windows for some reason
    else:
        os.system(f"xdg-open {config.output_file}")
