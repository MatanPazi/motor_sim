import os
from manim import *

class HelloWorld(Scene):
    def construct(self):
        text = Text("Hello, Manim!")
        self.play(Write(text))
        self.wait(2)

if __name__ == "__main__":
    from manim import config

    # Optional: Set output quality here (low or high)
    config.quality = "low_quality"    # Change to "high_quality" for higher resolution
    config.media_dir = os.getcwd()    # Optional: Set output directory

    # Render the scene
    HelloWorld().render()
    # Automatically open the output file
    os.system(f"xdg-open {config.output_file}")