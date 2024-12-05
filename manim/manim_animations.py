import os
import platform
from manim import *
import numpy as np

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
        # Define initial texts and equations
        simplified_text = Text("Simplified model: ").scale(0.9).to_edge(UP).shift(LEFT * 2)
        simplified_eq = MathTex("V = i \\cdot R + L \\cdot \\frac{di}{dt}").scale(1.1).next_to(simplified_text, RIGHT)

        ohms_law_text = Text("Ohm's law: ").scale(0.9).next_to(simplified_text, DOWN, aligned_edge=LEFT, buff=0.5)
        ohms_law_eq = MathTex("V = i \\cdot R").scale(1.1).next_to(ohms_law_text, RIGHT)

        faradays_law_text = Text("Faraday's law: ").scale(0.9).next_to(ohms_law_text, DOWN, aligned_edge=LEFT, buff=0.5)
        faradays_law_eq = MathTex("V = \\frac{d\\lambda}{dt}").scale(1.1).next_to(faradays_law_text, RIGHT)

        # Final equation with bold and green formatting
        final_eq = MathTex(
            r"\boldsymbol{V = i \cdot R + \frac{d\lambda}{dt}}").scale(1.5).next_to(faradays_law_eq, DOWN, buff=0.5)

        # Step 1: Show "Simplified model" and its equation
        self.play(Write(simplified_text))
        self.play(Write(simplified_eq))
        self.wait(2)

        # Step 2: Show "Ohm's law" and its equation
        self.play(Write(ohms_law_text))
        self.play(Write(ohms_law_eq))
        self.wait(1)

        # Step 3: Show "Faraday's law" and its equation
        self.play(Write(faradays_law_text))
        self.play(Write(faradays_law_eq))
        self.wait(1)

        # Step 4: Show the final equation at the center in bold and green
        self.play(Write(final_eq))
        self.wait(4)

        # Fade out all the previous texts and equations simultaneously
        self.play(
            *[FadeOut(mob) for mob in self.mobjects]
        )

        # Show the new equation "λ = L ⋅ i"
        new_eq = MathTex(r"\lambda = L \cdot i").scale(1.8).move_to(UP)
        self.play(Write(new_eq))
        self.wait(2)

        # Add "+ ψ" to the equation
        updated_eq = MathTex(r"\lambda = L \cdot i + \psi").scale(1.8).move_to(UP)
        self.play(Transform(new_eq, updated_eq))
        self.wait(2)                

        # Add d/dt to both sides of the equation
        derivative_eq = MathTex(r"\frac{d\lambda}{dt} = \frac{d}{dt}(L \cdot i + \psi)").scale(1.8).move_to(UP)
        self.play(Transform(new_eq, derivative_eq))
        self.wait(2)

        # Highlight 'i'
        circle_i = Circle(color=RED, radius=0.45).move_to(derivative_eq[0][13])  # Adjust index for the position of 'i'
        self.play(Create(circle_i))
        self.wait(2)

        # Fade out the circle after highlighting
        self.play(FadeOut(circle_i))

        # Highlight 'L'
        circle_L = Circle(color=RED, radius=0.45).move_to(derivative_eq[0][11])  # Adjust index for the position of 'L'
        self.play(Create(circle_L))
        self.wait(2)

        # Fade out the circle after highlighting
        self.play(FadeOut(circle_L))

        # Highlight 'psi'
        circle_psi = Circle(color=RED, radius=0.45).move_to(derivative_eq[0][15])  # Adjust index for the position of 'psi'
        self.play(Create(circle_psi))
        self.wait(2)

        # Fade out the circle after highlighting
        self.play(FadeOut(circle_psi))
        self.wait(2)

        # Fade out all the previous texts and equations simultaneously
        self.play(
            *[FadeOut(mob) for mob in self.mobjects]
        )

        # Display the new equation
        new_voltage_eq = MathTex(
            r"V = i \cdot r + L \cdot \frac{di}{dt} + i \cdot \frac{dL}{dt} + \frac{d\lambda}{dt}"
        ).scale(1.8).move_to(UP)

        self.play(Write(new_voltage_eq))
        self.wait(2)

        # Fade out all the previous texts and equations simultaneously
        self.play(
            *[FadeOut(mob) for mob in self.mobjects]
        )        

        # Display the first equation
        dL_dt_eq = MathTex(
            r"\frac{dL}{dt} = \frac{dL}{d\theta} \cdot \frac{d\theta}{dt} = \frac{dL}{d\theta} \cdot \omega"
        ).scale(1.8).to_edge(UP)

        # Display the second equation below the first
        dlambda_dt_eq = MathTex(
            r"\frac{d\lambda}{dt} = \frac{d\lambda}{d\theta} \cdot \frac{d\theta}{dt} = \frac{d\lambda}{d\theta} \cdot \omega"
        ).scale(1.8).next_to(dL_dt_eq, DOWN, buff=0.8)

        # Animate the equations
        self.play(Write(dL_dt_eq))
        self.play(Write(dlambda_dt_eq))
        self.wait(4)       

        # Fade out all the previous texts and equations simultaneously
        self.play(
            *[FadeOut(mob) for mob in self.mobjects]
        )             

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

        self.play(FadeOut(arrow1), FadeOut(arrow2), FadeOut(arrow3))
        self.wait(2)

        # Define the transformed equations with V_n already present and emphasized in red
        transformed_eq1 = MathTex(
            r"V_a = i_a \cdot r + L_a \cdot \frac{di_a}{dt} + L_{ab} \cdot \frac{di_b}{dt} + L_{ac} \cdot \frac{di_c}{dt} + \left( i_a \cdot \frac{dL_a}{d\theta} + i_b \cdot \frac{dL_{ab}}{d\theta} + i_c \cdot \frac{dL_{ac}}{d\theta} + \frac{d\lambda_a}{d\theta} \right) \cdot \omega + V_n"
        ).scale(0.65).move_to(eq1.get_center())

        transformed_eq2 = MathTex(
            r"V_b = i_b \cdot r + L_b \cdot \frac{di_b}{dt} + L_{ab} \cdot \frac{di_a}{dt} + L_{bc} \cdot \frac{di_c}{dt} + \left( i_b \cdot \frac{dL_b}{d\theta} + i_a \cdot \frac{dL_{ab}}{d\theta} + i_c \cdot \frac{dL_{bc}}{d\theta} + \frac{d\lambda_b}{d\theta} \right) \cdot \omega + V_n"
        ).scale(0.65).move_to(eq2.get_center())

        transformed_eq3 = MathTex(
            r"V_c = i_c \cdot r + L_c \cdot \frac{di_c}{dt} + L_{ac} \cdot \frac{di_a}{dt} + L_{bc} \cdot \frac{di_b}{dt} + \left( i_c \cdot \frac{dL_c}{d\theta} + i_b \cdot \frac{dL_{bc}}{d\theta} + i_a \cdot \frac{dL_{ac}}{d\theta} + \frac{d\lambda_c}{d\theta} \right) \cdot \omega + V_n"
        ).scale(0.65).move_to(eq3.get_center())
        
        # Add highlighting of V_n
        transformed_eq1[0][-2:].set_color(RED)
        transformed_eq2[0][-2:].set_color(RED)
        transformed_eq3[0][-2:].set_color(RED)

        # Animate the transformations and emphasize the already included V_n
        self.play(TransformMatchingShapes(eq1, transformed_eq1), TransformMatchingShapes(eq2, transformed_eq2), TransformMatchingShapes(eq3, transformed_eq3))
        self.wait(5)

        transformed_eq1[0][-2:].set_color(WHITE)
        transformed_eq2[0][-2:].set_color(WHITE)
        transformed_eq3[0][-2:].set_color(WHITE)        

        # Define the target equations
        target_eq1 = MathTex(
            r"L_a \cdot \frac{di_a}{dt} + L_{ab} \cdot \frac{di_b}{dt} + L_{ac} \cdot \frac{di_c}{dt} = "
            r"V_a - i_a \cdot r + \left( i_a \cdot \frac{dL_a}{d\theta} + i_b \cdot \frac{dL_{ab}}{d\theta} + i_c \cdot \frac{dL_{ac}}{d\theta} + \frac{d\lambda_a}{d\theta} \right) \cdot \omega + V_n"
        ).scale(0.65).move_to(transformed_eq1.get_center())

        target_eq2 = MathTex(
            r"L_b \cdot \frac{di_b}{dt} + L_{ab} \cdot \frac{di_a}{dt} + L_{bc} \cdot \frac{di_c}{dt} = "
            r"V_b - i_b \cdot r + \left( i_b \cdot \frac{dL_b}{d\theta} + i_a \cdot \frac{dL_{ab}}{d\theta} + i_c \cdot \frac{dL_{bc}}{d\theta} + \frac{d\lambda_b}{d\theta} \right) \cdot \omega + V_n"
        ).scale(0.65).move_to(transformed_eq2.get_center())

        target_eq3 = MathTex(
            r"L_c \cdot \frac{di_c}{dt} + L_{ac} \cdot \frac{di_a}{dt} + L_{bc} \cdot \frac{di_b}{dt} = "
            r"V_c - i_c \cdot r + \left( i_c \cdot \frac{dL_c}{d\theta} + i_b \cdot \frac{dL_{cb}}{d\theta} + i_a \cdot \frac{dL_{ac}}{d\theta} + \frac{d\lambda_c}{d\theta} \right) \cdot \omega + V_n"
        ).scale(0.65).move_to(transformed_eq3.get_center())

        # Apply transformations
        self.play(
            TransformMatchingShapes(transformed_eq1, target_eq1),
            TransformMatchingShapes(transformed_eq2, target_eq2),
            TransformMatchingShapes(transformed_eq3, target_eq3),
            run_time=3
        )        
        
        self.wait(4)
        
        self.play(
            *[FadeOut(mob) for mob in self.mobjects]
        )  

        simplified_eq = MathTex(
            r"L \cdot \frac{di}{dt} = V - i \cdot r + \omega \cdot \left( i \cdot \frac{dL}{d\theta} - \frac{d\lambda}{d\theta} \right) + V_n"
        ).scale(1).move_to(UP)

        self.play(Write(simplified_eq))

        self.wait(4)
        
        # Add underbraces directly to the existing equation
        underbrace_L = Brace(simplified_eq[0][0], DOWN, buff=0.2)
        text_L = underbrace_L.get_text("A")
        text_L.set_color(RED).set_weight(BOLD)

        underbrace_di = Brace(simplified_eq[0][2:6], DOWN, buff=0.2)
        text_di = underbrace_di.get_text("x")
        text_di.set_color(RED).set_weight(BOLD)

        underbrace_rhs = Brace(simplified_eq[0][8:31], DOWN, buff=0.2)
        text_rhs = underbrace_rhs.get_text("b")
        text_rhs.set_color(RED).set_weight(BOLD)

        # Animate
        self.play(Create(underbrace_L), Write(text_L))
        self.wait(1)
        self.play(Create(underbrace_di), Write(text_di))
        self.wait(1)
        self.play(Create(underbrace_rhs), Write(text_rhs))
        self.wait(5)    


      

class SineWaves120ChgAmp(Scene):
    def construct(self):
        # Create Axes for the plot
        axes = Axes(
            x_range=[0, 2 * PI, PI / 2], y_range=[-2, 2, 1],
            x_length=7, y_length=4.5,
            axis_config={"include_tip": False},
        ).to_edge(UP)

        # Define the sine waves
        sine1 = axes.plot(lambda x: np.sin(1.5 * x), color=RED, x_range=[0, 2 * PI])
        sine2 = axes.plot(lambda x: np.sin(1.5 * x - 2 * PI / 3), color=GREEN, x_range=[0, 2 * PI])
        sine3 = axes.plot(lambda x: np.sin(1.5 * x + 2 * PI / 3), color=BLUE, x_range=[0, 2 * PI])

        # Add labels
        label1 = MathTex("y_1 = \\sin(x)", color=RED).scale(0.8).next_to(axes, 6*LEFT)
        label2 = MathTex("y_2 = \\sin(x - 120^\\circ)", color=GREEN).scale(0.8).next_to(label1, DOWN, aligned_edge=LEFT)
        label3 = MathTex("y_3 = \\sin(x + 120^\\circ)", color=BLUE).scale(0.8).next_to(label2, DOWN, aligned_edge=LEFT)

        # Add the equation for the sum
        sum_eq = MathTex("y_1 + y_2 + y_3 = 0").scale(1.2).next_to(axes, DOWN)

        # Display the initial waves and sum equation
        self.play(Create(axes), Write(label1), Write(label2), Write(label3))
        self.play(Create(sine1), Create(sine2), Create(sine3))
        self.wait(1)
        self.play(Write(sum_eq))
        self.wait(2)

        # Increase the amplitude of one wave and update the equation
        modified_sine1 = axes.plot(lambda x: 1.5*np.sin(1.5 * x), color=RED, x_range=[0, 2 * PI])
        new_sum_eq = MathTex("y_1 + y_2 + y_3 \\neq 0").move_to(sum_eq)

        self.play(Transform(sine1, modified_sine1), Transform(sum_eq, new_sum_eq))
        self.wait(4)




class MatrixVectorMultiplication(Scene):
    def construct(self):
        # Define the matrix with symbolic variables
        matrix = MathTex(
            r"\begin{bmatrix}"
            r"L_{aa} & L_{ab} & L_{ac} & 1 \\"
            r"L_{ab} & L_{bb} & L_{bc} & 1 \\"
            r"L_{ac} & L_{bc} & L_{cc} & 1 \\"
            r"1 & 1 & 1 & 0"
            r"\end{bmatrix}"
        )
        
        vector = MathTex(
            r"\begin{bmatrix}"
            r"\frac{di_a}{dt} \\ \frac{di_b}{dt} \\ \frac{di_c}{dt} \\ V_n"
            r"\end{bmatrix}"
        )

        
        result = MathTex(
            r"\begin{bmatrix}"
            r"V_a - i_a R - \text{bemf}_a - \dot{L}_{aa} i_a - \dot{L}_{ab} i_b - \dot{L}_{ac} i_c \\"
            r"V_b - i_b R - \text{bemf}_b - \dot{L}_{ab} i_a - \dot{L}_{bb} i_b - \dot{L}_{bc} i_c \\"
            r"V_c - i_c R - \text{bemf}_c - \dot{L}_{ac} i_a - \dot{L}_{bc} i_b - \dot{L}_{cc} i_c \\"
            r"0"
            r"\end{bmatrix}"
        ) 

        
        # Equation derived from last row
        equation = MathTex(
            r"\frac{di_a}{dt} + \frac{di_b}{dt} + \frac{di_c}{dt} = 0"
        ).scale(1)

        # Position the matrix, vector, and result
        matrix.scale(0.85).to_edge(LEFT + UP)
        multiply = MathTex(" \\times ").scale(0.6).next_to(matrix, RIGHT, buff=0.15)
        vector.scale(0.85).next_to(multiply, RIGHT, buff=0.15)
        equals = MathTex("=").scale(0.6).next_to(vector, RIGHT, buff=0.15)
        result.scale(0.85).next_to(equals, RIGHT, buff=0.15)

        # Animate the setup
        self.play(Write(matrix), Write(vector), Write(multiply))
        self.play(Write(equals), Write(result))
        self.wait(1)

        # Highlight the last row and vector
        last_row = SurroundingRectangle(matrix[0][-8:-4], color=YELLOW)  # Adjust to match indices for the last row
        vector_rect = SurroundingRectangle(vector[-1], color=YELLOW)
        self.play(Create(last_row), Create(vector_rect))
        self.wait(1)

        # Transform last row and vector into the equation
        self.play(
            TransformFromCopy(matrix[0][-8:-4], equation[0][:7]),  # Transform last row coefficients
            TransformFromCopy(vector[0], equation[0][8:]),      # Transform vector elements
        )        
        self.wait(2)


class VectorProjections(Scene):
    def construct(self):
        # Axes setup
        axes = Axes(
            x_range=[-2, 2, 0.5],
            y_range=[-2, 2, 0.5],
            axis_config={"color": GREY},
        )
        d_label = axes.get_x_axis_label("d", direction=RIGHT)
        q_label = axes.get_y_axis_label("q", direction=UP)

        # Labels for axes
        self.play(Create(axes), Write(d_label), Write(q_label))

        # Vector Is
        magnitude = 1.5
        is_vector = Arrow(
            start=axes.c2p(0, 0),
            end=axes.c2p(magnitude * np.cos(PI / 6), magnitude * np.sin(PI / 6)),
            color=WHITE
        )
        is_label = MathTex("I_s").next_to(is_vector.get_end(), UP + RIGHT, buff=0.1)
        # Update the label position with the vector
        def update_label(label):
            label.next_to(is_vector.get_end(), UP + RIGHT, buff=0.1)

        # Add the updater for the label
        is_label.add_updater(update_label)        

        # Projections
        # Correct projection for Id (on x-axis)
        id_line = always_redraw(lambda: DashedLine(
            start=axes.c2p(axes.p2c(is_vector.get_end())[0], 0),
            end=axes.c2p(axes.p2c(is_vector.get_end())[0], axes.p2c(is_vector.get_end())[1]),
            color=BLUE
        ))
        id_label = always_redraw(lambda: MathTex("I_d", color=BLUE).next_to(id_line, DOWN, buff=0.2))

        # Correct projection for Iq (on y-axis)
        iq_line = always_redraw(lambda: DashedLine(
            start=axes.c2p(0, axes.p2c(is_vector.get_end())[1]),
            end=axes.c2p(axes.p2c(is_vector.get_end())[0], axes.p2c(is_vector.get_end())[1]),
            color=RED,
        ))
        iq_label = always_redraw(lambda: MathTex("I_q", color=RED).next_to(iq_line, LEFT, buff=0.2))

        # Animate Is moving between 30 and 60 degrees
        current_angle = ValueTracker(30 * DEGREES)

        def update_vector(vector):
            angle = current_angle.get_value()
            vector.put_start_and_end_on(
                axes.c2p(0, 0),
                axes.c2p(magnitude * np.cos(angle), magnitude * np.sin(angle))
            )

        is_vector.add_updater(update_vector)

        # Create arrows for Id and Iq
        id_vector = Arrow(
            start=axes.c2p(0, 0),
            end=axes.c2p(magnitude * np.cos(PI / 6), 0),
            color=BLUE,
            stroke_width=4  # Keep arrow width constant
        )
        iq_vector = Arrow(
            start=axes.c2p(0, 0),
            end=axes.c2p(0.01, magnitude * np.sin(PI / 6)),
            color=RED,
            max_tip_length_to_length_ratio = 5
        )        

        def update_vector_id(vector):
            angle = current_angle.get_value()
            vector.put_start_and_end_on(
                axes.c2p(0, 0),
                axes.c2p(magnitude * np.cos(angle), 0)
            )
        def update_vector_iq(vector):
            angle = current_angle.get_value()
            vector.put_start_and_end_on(
                axes.c2p(0, 0),
                axes.c2p(0.01, magnitude * np.sin(angle))
            )            

        id_vector.add_updater(update_vector_id)
        iq_vector.add_updater(update_vector_iq)      

        # Update Is and projections
        self.play(Create(is_vector), Write(is_label), Create(id_line), Write(id_label), Create(iq_line), Write(iq_label), Create(id_vector), Create(iq_vector))

        # Animate the angle oscillating between 30 and 60 degrees
        self.play(current_angle.animate.set_value(60 * DEGREES), run_time=6, rate_func=there_and_back)

        self.add(is_vector)
        self.add(id_vector)
        self.add(iq_vector)
        self.wait(2)  # Let the vector move for 4 seconds

        # Stop the animation and hold the final scene
        is_vector.remove_updater(update_vector)
        id_vector.remove_updater(update_vector_id)
        iq_vector.remove_updater(update_vector_iq)
        self.wait()




class ThreePhaseSineWaves(Scene):
    def construct(self):
        # Set up the axes
        axes = Axes(
            x_range=[0, 6*np.pi, np.pi/2],
            y_range=[-2.5, 2.5, 0.5],
            axis_config={"color": BLUE},
            x_length=10.0
        )
        axes_labels = axes.get_axis_labels(x_label="t", y_label="V")

        # Define the sine waves
        def sine_wave(x, phase=0):
            return np.sin(x + phase)

        # Create the initial three sine waves
        graph1 = axes.plot(lambda x: sine_wave(x), color=RED)
        graph2 = axes.plot(lambda x: sine_wave(x, 2*np.pi/3), color=GREEN)
        graph3 = axes.plot(lambda x: sine_wave(x, 4*np.pi/3), color=BLUE)

        # Show the initial setup
        self.play(Create(axes), Create(axes_labels))
        self.play(Create(graph1), Create(graph2), Create(graph3))
        self.wait(1)

        # Add crossed line and label for phase voltages
        start_point_1 = axes.c2p(0, 1)
        end_point_1 = axes.c2p(6*np.pi, 1)
        dashed_line_1 = DashedLine(start_point_1, end_point_1, color=WHITE)
        label_1 = MathTex("\\frac{V_{bus}}{2}").next_to(start_point_1, LEFT).scale(0.6)

        # Add crossed line and label for phase voltages
        start_point_2 = axes.c2p(0, 2)
        end_point_2 = axes.c2p(6*np.pi, 2)
        dashed_line_2 = DashedLine(start_point_2, end_point_2, color=WHITE)
        label_2 = MathTex("V_{bus}").next_to(start_point_2, np.array((-1.0, 0.1, 0.0))).scale(0.6)

        self.play(Create(dashed_line_1), Write(label_1), Create(dashed_line_2), Write(label_2))        
        self.wait(1)

        # Remove one sine wave and add the difference function
        self.play(FadeOut(graph3))
        diff_graph = axes.plot(lambda x: sine_wave(x) - sine_wave(x, 2*np.pi/3), color=YELLOW)

        # Add crossed line and label for phase voltages
        start_point_3 = axes.c2p(0, 1.732)
        end_point_3 = axes.c2p(6*np.pi, 1.732)
        dashed_line_3 = DashedLine(start_point_3, end_point_3, color=WHITE)
        label_3 = MathTex("\\frac{V_{bus}}{1.154}").next_to(start_point_3, LEFT).scale(0.6)

        self.play(Create(diff_graph), Create(dashed_line_3), Write(label_3))
        self.wait(4)    

        # Increase amplitude of original sine waves
        def increased_sine_wave(x, phase=0):
            return (2/np.sqrt(3)) * np.sin(x + phase)

        new_graph1 = axes.plot(lambda x: increased_sine_wave(x), color=RED)
        new_graph2 = axes.plot(lambda x: increased_sine_wave(x, 2*np.pi/3), color=GREEN)
        new_diff_graph = axes.plot(lambda x: increased_sine_wave(x) - increased_sine_wave(x, 2*np.pi/3), color=YELLOW)

        self.play(
            ReplacementTransform(graph1, new_graph1),
            ReplacementTransform(graph2, new_graph2),
            ReplacementTransform(diff_graph, new_diff_graph)
        )
        self.wait(4)    

        # Add third harmonic
        def harmonic_sine_wave(x, phase=0):
            return (2/np.sqrt(3)) * (np.sin(x + phase) + (1/6) * np.sin(3*(x + phase)))

        harmonic = axes.plot(lambda x: sine_wave(3*x) * 1.154 / 6, color=PURPLE)
        harmonic_graph1 = axes.plot(lambda x: harmonic_sine_wave(x), color=RED)
        harmonic_graph2 = axes.plot(lambda x: harmonic_sine_wave(x, 2*np.pi/3), color=GREEN)
        harmonic_diff_graph = axes.plot(lambda x: harmonic_sine_wave(x) - harmonic_sine_wave(x, 2*np.pi/3), color=YELLOW)

        # Add crossed line and label for phase voltages
        start_point_4 = axes.c2p(0, 1.154/6)
        end_point_4 = axes.c2p(6*np.pi, 1.154/6)
        dashed_line_4 = DashedLine(start_point_4, end_point_4, color=WHITE)
        label_4 = MathTex("\\frac{1.154 * V_{bus}}{6}").next_to(start_point_4, np.array((-0.01, 0.0, 0.0))).scale(0.6)        

        self.play(Create(harmonic))
        self.wait(1)
        self.play(Create(dashed_line_4), Write(label_4))
        self.wait(2)

        self.play(
            ReplacementTransform(new_graph1, harmonic_graph1),
            ReplacementTransform(new_graph2, harmonic_graph2),
            ReplacementTransform(new_diff_graph, harmonic_diff_graph),
        )
        self.wait(4)

        harmonic_new = axes.plot(lambda x: 1.2*sine_wave(3*x) * 1.154 / 6, color=PURPLE)
        harmonic_graph1_new = axes.plot(lambda x: 1.2*harmonic_sine_wave(x), color=RED)
        harmonic_graph2_new = axes.plot(lambda x: 1.2*harmonic_sine_wave(x, 2*np.pi/3), color=GREEN)
        harmonic_diff_graph_new = axes.plot(lambda x: 1.2*harmonic_sine_wave(x) - 1.2*harmonic_sine_wave(x, 2*np.pi/3), color=YELLOW)

        self.play(
            ReplacementTransform(harmonic_graph1, harmonic_graph1_new),
            ReplacementTransform(harmonic_graph2, harmonic_graph2_new),
            ReplacementTransform(harmonic_diff_graph, harmonic_diff_graph_new),
            ReplacementTransform(harmonic, harmonic_new),
        )
        self.wait(4)



if __name__ == "__main__":
    from manim import config

    # Optional: Set output quality here (low or high)
    config.quality = "low_quality"    # Change to "high_quality" for higher resolution
    config.media_dir = os.getcwd()    # Optional: Set output directory

    # Render the scene
    MatrixVectorMultiplication().render()

    # Automatically open the output file
    if platform.system() == 'Windows':
        os.system(f"start {config.output_file}")
        input("Press Enter to exit...")  # Prevents the script from closing immediately. Need in Windows for some reason
    else:
        os.system(f"xdg-open {config.output_file}")
