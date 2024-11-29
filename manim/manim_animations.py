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





class ElectricalModel(Scene):
    def construct(self):
        # # Define initial texts and equations
        # simplified_text = Text("Simplified model: ").scale(0.9).to_edge(UP).shift(LEFT * 2)
        # simplified_eq = MathTex("V = i \\cdot R + L \\cdot \\frac{di}{dt}").scale(1.1).next_to(simplified_text, RIGHT)

        # ohms_law_text = Text("Ohm's law: ").scale(0.9).next_to(simplified_text, DOWN, aligned_edge=LEFT, buff=0.5)
        # ohms_law_eq = MathTex("V = i \\cdot R").scale(1.1).next_to(ohms_law_text, RIGHT)

        # faradays_law_text = Text("Faraday's law: ").scale(0.9).next_to(ohms_law_text, DOWN, aligned_edge=LEFT, buff=0.5)
        # faradays_law_eq = MathTex("V = \\frac{d\\lambda}{dt}").scale(1.1).next_to(faradays_law_text, RIGHT)

        # # Final equation with bold and green formatting
        # final_eq = MathTex(
        #     r"\boldsymbol{V = i \cdot R + \frac{d\lambda}{dt}}").scale(1.5).next_to(faradays_law_eq, DOWN, buff=0.5)

        # # Step 1: Show "Simplified model" and its equation
        # self.play(Write(simplified_text))
        # self.play(Write(simplified_eq))
        # self.wait(2)

        # # Step 2: Show "Ohm's law" and its equation
        # self.play(Write(ohms_law_text))
        # self.play(Write(ohms_law_eq))
        # self.wait(1)

        # # Step 3: Show "Faraday's law" and its equation
        # self.play(Write(faradays_law_text))
        # self.play(Write(faradays_law_eq))
        # self.wait(1)

        # # Step 4: Show the final equation at the center in bold and green
        # self.play(Write(final_eq))
        # self.wait(4)

        # # Fade out all the previous texts and equations simultaneously
        # self.play(
        #     *[FadeOut(mob) for mob in self.mobjects]
        # )

        # # Show the new equation "λ = L ⋅ i"
        # new_eq = MathTex(r"\lambda = L \cdot i").scale(1.8).move_to(UP)
        # self.play(Write(new_eq))
        # self.wait(2)

        # # Add "+ ψ" to the equation
        # updated_eq = MathTex(r"\lambda = L \cdot i + \psi").scale(1.8).move_to(UP)
        # self.play(Transform(new_eq, updated_eq))
        # self.wait(2)                

        # # Add d/dt to both sides of the equation
        # derivative_eq = MathTex(r"\frac{d\lambda}{dt} = \frac{d}{dt}(L \cdot i + \psi)").scale(1.8).move_to(UP)
        # self.play(Transform(new_eq, derivative_eq))
        # self.wait(2)

        # # Highlight 'i'
        # circle_i = Circle(color=RED, radius=0.45).move_to(derivative_eq[0][13])  # Adjust index for the position of 'i'
        # self.play(Create(circle_i))
        # self.wait(2)

        # # Fade out the circle after highlighting
        # self.play(FadeOut(circle_i))

        # # Highlight 'L'
        # circle_L = Circle(color=RED, radius=0.45).move_to(derivative_eq[0][11])  # Adjust index for the position of 'L'
        # self.play(Create(circle_L))
        # self.wait(2)

        # # Fade out the circle after highlighting
        # self.play(FadeOut(circle_L))

        # # Highlight 'psi'
        # circle_psi = Circle(color=RED, radius=0.45).move_to(derivative_eq[0][15])  # Adjust index for the position of 'psi'
        # self.play(Create(circle_psi))
        # self.wait(2)

        # # Fade out the circle after highlighting
        # self.play(FadeOut(circle_psi))
        # self.wait(2)

        # # Fade out all the previous texts and equations simultaneously
        # self.play(
        #     *[FadeOut(mob) for mob in self.mobjects]
        # )

        # # Display the new equation
        # new_voltage_eq = MathTex(
        #     r"V = i \cdot r + L \cdot \frac{di}{dt} + i \cdot \frac{dL}{dt} + \frac{d\lambda}{dt}"
        # ).scale(1.8).move_to(UP)

        # self.play(Write(new_voltage_eq))
        # self.wait(2)

        # # Fade out all the previous texts and equations simultaneously
        # self.play(
        #     *[FadeOut(mob) for mob in self.mobjects]
        # )        

        # # Display the first equation
        # dL_dt_eq = MathTex(
        #     r"\frac{dL}{dt} = \frac{dL}{d\theta} \cdot \frac{d\theta}{dt} = \frac{dL}{d\theta} \cdot \omega"
        # ).scale(1.8).to_edge(UP)

        # # Display the second equation below the first
        # dlambda_dt_eq = MathTex(
        #     r"\frac{d\lambda}{dt} = \frac{d\lambda}{d\theta} \cdot \frac{d\theta}{dt} = \frac{d\lambda}{d\theta} \cdot \omega"
        # ).scale(1.8).next_to(dL_dt_eq, DOWN, buff=0.8)

        # # Animate the equations
        # self.play(Write(dL_dt_eq))
        # self.play(Write(dlambda_dt_eq))
        # self.wait(4)       

        # # Fade out all the previous texts and equations simultaneously
        # self.play(
        #     *[FadeOut(mob) for mob in self.mobjects]
        # )             

        # Display the new equation
        voltage_eq = MathTex(
            r"V = i \cdot r + L \cdot \frac{di}{dt} + i \cdot \frac{dL}{d\theta} \cdot \omega + \frac{d\lambda}{d\theta} \cdot \omega"
        ).scale(1.5)

        print('Here!     ',voltage_eq[0])

        self.play(Write(voltage_eq))
        self.wait(2)

        # Highlight 'dlambda/dtheta'
        circle_dlambda_dtheta = Circle(color=RED, radius=1.1).move_to(voltage_eq[0][26])  # Adjust index for the position of 'L'

        # Add the text "bemf const" at the start of the arrow
        bemf_text = Text("bemf const", color=RED).scale(1).next_to(circle_dlambda_dtheta, DOWN, buff=0.2)

        # Animate the circle and text
        self.play(Create(circle_dlambda_dtheta), run_time=0.5)
        self.play(Write(bemf_text), run_time=1)
        self.wait(2)      

        self.play(
            *[FadeOut(mob) for mob in self.mobjects]
        )              

        # Define the three equations
        eq1 = MathTex(
            r"V_a = i_a \cdot r + L_a \cdot \frac{di_a}{dt} + L_{ab} \cdot \frac{di_b}{dt} + L_{ac} \cdot \frac{di_c}{dt} + \left( i_a \cdot \frac{dL_a}{d\theta} + i_b \cdot \frac{dL_{ab}}{d\theta} + i_c \cdot \frac{dL_{ac}}{d\theta} + \frac{d\lambda_a}{d\theta} \right) \cdot \omega"
        ).scale(0.7).move_to(UP * 2)

        eq2 = MathTex(
            r"V_b = i_b \cdot r + L_b \cdot \frac{di_b}{dt} + L_{ab} \cdot \frac{di_a}{dt} + L_{bc} \cdot \frac{di_c}{dt} + \left( i_b \cdot \frac{dL_b}{d\theta} + i_a \cdot \frac{dL_{ab}}{d\theta} + i_c \cdot \frac{dL_{bc}}{d\theta} + \frac{d\lambda_b}{d\theta} \right) \cdot \omega"
        ).scale(0.7).next_to(eq1, DOWN, buff=0.8)

        eq3 = MathTex(
            r"V_c = i_c \cdot r + L_c \cdot \frac{di_c}{dt} + L_{ac} \cdot \frac{di_a}{dt} + L_{bc} \cdot \frac{di_b}{dt} + \left( i_c \cdot \frac{dL_c}{d\theta} + i_b \cdot \frac{dL_{bc}}{d\theta} + i_a \cdot \frac{dL_{ac}}{d\theta} + \frac{d\lambda_c}{d\theta} \right) \cdot \omega"
        ).scale(0.7).next_to(eq2, DOWN, buff=0.8)

        # Add all three equations to the scene
        self.play(Write(eq1), Write(eq2), Write(eq3), run_time=4)
        self.wait(4)

        # Create arrows between the equations
        arrow1 = DoubleArrow(color=GREEN, start=eq1.get_center(), end=eq2.get_center(), tip_length=0.4, stroke_width=9)
        arrow2 = DoubleArrow(color=GREEN, start=eq2.get_center(), end=eq3.get_center(), tip_length=0.4, stroke_width=9)

        # Adjust the third arrow to move it to the side
        arrow3 = DoubleArrow(
            color=GREEN, 
            start=eq3.get_center() + LEFT * 2,  # Offset to the left
            end=eq1.get_center() + LEFT * 2,   # Offset to the right
            tip_length=0.4,
            stroke_width=9
        )

        # Add arrows to the scene
        self.play(Create(arrow1), Create(arrow2), Create(arrow3), run_time=0.2)
        self.wait(4)
    

      













if __name__ == "__main__":
    from manim import config

    # Optional: Set output quality here (low or high)
    config.quality = "low_quality"    # Change to "high_quality" for higher resolution
    config.media_dir = os.getcwd()    # Optional: Set output directory

    # Render the scene
    ElectricalModel().render()

    # Automatically open the output file
    if platform.system() == 'Windows':
        os.system(f"start {config.output_file}")
        input("Press Enter to exit...")  # Prevents the script from closing immediately. Need in Windows for some reason
    else:
        os.system(f"xdg-open {config.output_file}")
