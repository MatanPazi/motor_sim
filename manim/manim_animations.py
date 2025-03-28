import os
import platform
from manim import *
import numpy as np
from manim import Matrix


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
        self.wait(7)

        # Step 4: Show the final equation at the center in bold and green
        self.play(Write(final_eq))
        self.wait(8)

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
        matrix = Matrix([
            ["L_{aa}", "L_{ab}", "L_{ac}", "1"],
            ["L_{ab}", "L_{bb}", "L_{bc}", "1"],
            ["L_{ac}", "L_{bc}", "L_{cc}", "1"],
            ["1", "1", "1", "0"]
        ])
        
        vector = MathTex(
            r"\begin{bmatrix}"
            r"\frac{d{i}_a}{dt} \\[0.2cm]"
            r"\frac{d{i}_b}{dt} \\[0.2cm]"
            r"\frac{d{i}_c}{dt} \\[0.2cm]"
            r"V_n"
            r"\end{bmatrix}"
        )

        result = MathTex(
            r"\begin{bmatrix}"
            r"V_a - i_a R - \text{bemf}_a - \dot{L}_{aa} i_a - \dot{L}_{ab} i_b - \dot{L}_{ac} i_c \\[0.2cm]"
            r"V_b - i_b R - \text{bemf}_b - \dot{L}_{ab} i_a - \dot{L}_{bb} i_b - \dot{L}_{bc} i_c \\[0.2cm]"
            r"V_c - i_c R - \text{bemf}_c - \dot{L}_{ac} i_a - \dot{L}_{bc} i_b - \dot{L}_{cc} i_c \\[0.2cm]"
            r"0"
            r"\end{bmatrix}"
        )

        # Equation derived from last row
        equation = MathTex(
            r"\frac{d{i}_a}{dt} + \frac{d{i}_b}{dt} + \frac{d{i}_c}{dt} = 0"
        ).scale(1)
        

        # Position the matrix, vector, and result
        matrix.scale(0.8).to_edge(LEFT + UP)
        multiply = MathTex(" \\times ").scale(0.6).next_to(matrix, RIGHT, buff=0.15)
        vector.scale(0.8).next_to(multiply, RIGHT, buff=0.15)
        equals = MathTex("=").scale(0.6).next_to(vector, RIGHT, buff=0.15)
        result.scale(0.8).next_to(equals, RIGHT, buff=0.15)

        # Animate the setup
        self.play(Write(matrix), Write(vector), Write(multiply), run_time = 1.5)
        self.play(Write(equals), Write(result), run_time = 1.5)

        # Add rectangle around the last column
        last_column_rect = SurroundingRectangle(matrix.get_columns()[-1], color=YELLOW, buff=0.2)
        self.play(Create(last_column_rect))
        self.wait(2)
        self.play(FadeOut(last_column_rect))

        # Highlight the last row and vector
        last_row = SurroundingRectangle(matrix.get_rows()[-1], color=YELLOW, buff=0.2)  # Adjust to match indices for the last row
        vector_rect = SurroundingRectangle(vector[0], color=YELLOW)
        self.play(Create(last_row), Create(vector_rect))
        self.wait(1)

        # Transform last row and vector into the equation
        self.play(
            TransformFromCopy(matrix[0][-8:-4], equation[0][:7]),  # Transform last row coefficients
            TransformFromCopy(vector[0], equation[0][7:]),      # Transform vector elements
        )        
        self.wait(5)


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



class CenterAlignedPWM(Scene):
    def construct(self):
        # Create axes
        axes = Axes(
            x_range=[0, 250, 50],
            y_range=[0, 125, 25],
            axis_config={"color": BLUE},
            x_axis_config={"numbers_to_include": range(0, 201, 50)},
            y_axis_config={"numbers_to_include": range(0, 101, 25)},
        ).scale(0.65).to_edge(UP, buff=1.0)
        
        # Add labels
        x_label = axes.get_x_axis_label("Time", direction=RIGHT)
        y_label = axes.get_y_axis_label("Counter", direction=2*UP)
        
        # Create timer
        timer = Integer(0).to_corner(UR, buff=2.0)
        timer_label = Text("Timer :").next_to(timer, LEFT)
        timer_group = VGroup(timer_label, timer)
        
        # Create triangular waveform
        triangle_points = [
            axes.c2p(0, 0),
            axes.c2p(100, 100),
            axes.c2p(200, 0)
        ]
        triangle = VMobject(color=RED)
        triangle.set_points_as_corners(triangle_points)
        
        # Create vertical lines and label for PWM period
        start_line = axes.get_vertical_line(axes.c2p(0, -30), color=YELLOW, stroke_width=8)
        end_line = axes.get_vertical_line(axes.c2p(200, -30), color=YELLOW, stroke_width=8)
        period_arrow = DoubleArrow(start=start_line.get_bottom(), end=end_line.get_bottom(), buff=0.1, color=YELLOW)
        period_label = Text("PWM Period", color=YELLOW).next_to(period_arrow, DOWN)
        pwm_group = VGroup(start_line, end_line, period_arrow, period_label)
        
        # Add elements to the scene
        self.add(axes, x_label, y_label, timer_group)
        self.wait(2)
        
        # Animate the waveform and timer
        self.play(
            Create(triangle),
            UpdateFromAlphaFunc(timer, lambda m, alpha: m.set_value(alpha * 200)),
            run_time=10,
            rate_func=linear
        )
        
        # Add PWM period indication after triangle is complete
        self.play(
            Create(pwm_group)
        )
        
        self.wait(2)

        self.play(FadeOut(pwm_group))

        # Add dashed horizontal line at y = 50
        dashed_line = DashedLine(
            start=axes.c2p(0, 50),
            end=axes.c2p(200, 50),
            color=GREEN,
            dash_length=0.1
        )

        # Create transistor state texts
        top_text = Text("Top Transistors: ", color=BLUE).scale(0.5)
        bottom_text = Text("Bottom Transistors: ", color=RED).scale(0.5)
        top_state = Text("OFF", weight=BOLD, color=BLUE).scale(0.5)
        bottom_state = Text("ON", weight=BOLD, color=RED).scale(0.5)

        # Position the texts inside the triangular waveform
        top_group = VGroup(top_text, top_state).arrange(RIGHT)
        bottom_group = VGroup(bottom_text, bottom_state).arrange(RIGHT)
        
        top_group.move_to(axes.c2p(100, 30))  # Adjust these coordinates as needed
        bottom_group.next_to(top_group, DOWN, buff=0.2, aligned_edge=LEFT)  # Adjust these coordinates as needed

        # Add point on x-axis at x = 20
        point = Dot(axes.c2p(20, 0), color=YELLOW)

        # Add elements to the scene
        self.play(Create(dashed_line))

        # Show the point
        self.play(Create(point))
        self.wait(1)

        # Move point upwards to connect with triangle and move timer below the plot
        self.play(
            point.animate.move_to(axes.c2p(20, 20)),
            timer.animate.set_value(20),
            timer_group.animate.next_to(axes, DOWN, buff=0.5),
            FadeIn(top_group),
            FadeIn(bottom_group),         
            run_time=1.5
        )

        self.wait(2)    

        # Function to update point position
        def update_point(mob, alpha):
            x = 20 + alpha * 180  # x moves from 20 to 200
            y = 100 - abs(x - 100)  # y follows triangle
            mob.move_to(axes.c2p(x, y))

        # Function to update timer
        def update_timer(mob, alpha):
            new_value = int(20 + alpha * 180)  # Increment from 20 to 200
            mob.set_value(new_value)

        # Function to update transistor states
        def update_states(alpha):
            x = 20 + alpha * 180
            y = 100 - abs(x - 100)
            if y > 50:
                top_state.become(Text("ON", weight=BOLD, color=BLUE).scale(0.5).next_to(top_text, RIGHT))
                bottom_state.become(Text("OFF", weight=BOLD, color=RED).scale(0.5).next_to(bottom_text, RIGHT))
            else:
                top_state.become(Text("OFF", weight=BOLD, color=BLUE).scale(0.5).next_to(top_text, RIGHT))
                bottom_state.become(Text("ON", weight=BOLD, color=RED).scale(0.5).next_to(bottom_text, RIGHT))       

        # Animate point moving along triangle, update timer and transistor states
        self.play(
            UpdateFromAlphaFunc(point, update_point),
            UpdateFromAlphaFunc(timer, update_timer),
            UpdateFromAlphaFunc(VGroup(), lambda m, alpha: update_states(alpha)),
            run_time=10,
            rate_func=linear
        )

        self.wait(2)


        # Add two new dashed horizontal lines
        lower_dashed_line = DashedLine(
            start=axes.c2p(0, 45),
            end=axes.c2p(200, 45),
            color=GREEN_A,
            dash_length=0.1
        )
        upper_dashed_line = DashedLine(
            start=axes.c2p(0, 55),
            end=axes.c2p(200, 55),
            color=GREEN_A,
            dash_length=0.1
        )

        # Function to update transistor states with new logic
        def update_states(alpha):
            x = 20 + alpha * 180
            y = 100 - abs(x - 100)
            if y > 55:
                top_state.become(Text("ON", weight=BOLD, color=BLUE).scale(0.5).next_to(top_text, RIGHT))
            else:
                top_state.become(Text("OFF", weight=BOLD, color=BLUE).scale(0.5).next_to(top_text, RIGHT))
            
            if y < 45:
                bottom_state.become(Text("ON", weight=BOLD, color=RED).scale(0.5).next_to(bottom_text, RIGHT))
            else:
                bottom_state.become(Text("OFF", weight=BOLD, color=RED).scale(0.5).next_to(bottom_text, RIGHT))


        # After the main animation, add new lines and reset point
        self.play(
            Create(lower_dashed_line),
            Create(upper_dashed_line),
            point.animate.move_to(axes.c2p(20, 20)),
            run_time=1.5
        )

        # Animate point moving along triangle with new state logic
        self.play(
            UpdateFromAlphaFunc(point, update_point),
            UpdateFromAlphaFunc(timer, update_timer),
            UpdateFromAlphaFunc(VGroup(), lambda m, alpha: update_states(alpha)),
            run_time=10,
            rate_func=linear
        )

        self.wait(2)        




class InductanceEquations(Scene):
    def construct(self):
        # Define constants
        Lq, Ld = 1.4, 0.8

        # First set of equations
        self.first_set(Lq, Ld)
        
        # Fade out everything
        self.play(*[FadeOut(mob) for mob in self.mobjects])
        
        # Second set of equations
        self.second_set(Lq, Ld)

    def first_set(self, Lq, Ld):
        # Create equations with colors
        eq1 = MathTex(r"L_{aa} = L_q \cos^2(\theta) + L_d \sin^2(\theta)", color=RED)
        eq2 = MathTex(r"L_{bb} = L_q \cos^2(\theta - \frac{2\pi}{3}) + L_d \sin^2(\theta - \frac{2\pi}{3})", color=GREEN)
        eq3 = MathTex(r"L_{cc} = L_q \cos^2(\theta + \frac{2\pi}{3}) + L_d \sin^2(\theta + \frac{2\pi}{3})", color=BLUE)

        equations = VGroup(eq1, eq2, eq3).arrange(DOWN, buff=0.3, aligned_edge=LEFT)

        # Show equations in the middle
        self.play(Write(equations))
        self.wait(2)

        # Make equations smaller and move to top
        self.play(
            equations.animate.scale(0.4).to_corner(UP, buff=0.5)
        )
        self.wait(1)

        # Create the plot
        axes = Axes(
            x_range=[0, 2*PI, PI/2],
            y_range=[0, 1.5, 0.5],
            axis_config={"include_tip": False},
            x_axis_config={"label_direction": DOWN},
            y_axis_config={"label_direction": LEFT},
            tips=False,
        )

        x_label = axes.get_x_axis_label(r"\theta").scale(0.7)
        y_label = axes.get_y_axis_label("L").scale(0.7)

        plot = VGroup(axes, x_label, y_label).scale(0.8).shift(DOWN * 0.5)

        self.play(Create(plot))

        # Define the functions
        def Laa(x): return Lq * np.cos(x)**2 + Ld * np.sin(x)**2
        def Lbb(x): return Lq * np.cos(x - 2*np.pi/3)**2 + Ld * np.sin(x - 2*np.pi/3)**2
        def Lcc(x): return Lq * np.cos(x + 2*np.pi/3)**2 + Ld * np.sin(x + 2*np.pi/3)**2

        # Create the graphs
        graph_aa = axes.plot(Laa, color=RED)
        graph_bb = axes.plot(Lbb, color=GREEN)
        graph_cc = axes.plot(Lcc, color=BLUE)

        # Add labels to the graphs
        label_aa = Text("Laa", color=RED, font_size=24).next_to(graph_aa.points[-1], UR)
        label_bb = Text("Lbb", color=GREEN, font_size=24).next_to(graph_bb.points[-1], DR)
        label_cc = Text("Lcc", color=BLUE, font_size=24).next_to(graph_cc.points[-1], UR)

        self.play(Create(graph_aa), Create(graph_bb), Create(graph_cc))
        self.play(Write(label_aa), Write(label_bb), Write(label_cc))

        # Add horizontal lines for Lq and Ld
        line_Lq = DashedLine(
            axes.c2p(0, Lq), axes.c2p(2*PI, Lq), 
            color=YELLOW, dash_length=0.1
        )
        line_Ld = DashedLine(
            axes.c2p(0, Ld), axes.c2p(2*PI, Ld), 
            color=YELLOW, dash_length=0.1
        )
        Lq_label = Text("Lq", color=YELLOW, font_size=24).next_to(line_Lq, LEFT)
        Ld_label = Text("Ld", color=YELLOW, font_size=24).next_to(line_Ld, LEFT)

        self.play(Create(line_Lq), Create(line_Ld), Write(Lq_label), Write(Ld_label))

        self.wait(3)

    def second_set(self, Lq, Ld):
        # Create new equations with colors
        eq1 = MathTex(r"L_{ab} = L_q \cos(\theta) \cos(\theta - \frac{2\pi}{3}) + L_d \sin(\theta) \sin(\theta - \frac{2\pi}{3})", color=RED)
        eq2 = MathTex(r"L_{ac} = L_q \cos(\theta) \cos(\theta + \frac{2\pi}{3}) + L_d \sin(\theta) \sin(\theta + \frac{2\pi}{3})", color=GREEN)
        eq3 = MathTex(r"L_{bc} = L_q \cos(\theta - \frac{2\pi}{3}) \cos(\theta + \frac{2\pi}{3}) + L_d \sin(\theta - \frac{2\pi}{3}) \sin(\theta + \frac{2\pi}{3})", color=BLUE)

        equations = VGroup(eq1, eq2, eq3).arrange(DOWN, buff=0.3, aligned_edge=LEFT)

        # Show equations in the middle
        self.play(Write(equations))
        self.wait(2)

        # Make equations smaller and move to top
        self.play(
            equations.animate.scale(0.4).to_corner(UP, buff=0.5)
        )
        self.wait(1)

        # Create the plot
        axes = Axes(
            x_range=[0, 2*PI, PI/2],
            y_range=[-1.0, 0.0, 0.5],
            axis_config={"include_tip": False},
            x_axis_config={"label_direction": DOWN},
            y_axis_config={"label_direction": LEFT},
            tips=False,
        ).scale(0.9)

        x_label = axes.get_x_axis_label(r"\theta").scale(0.7)
        y_label = axes.get_y_axis_label("L").scale(0.7)

        plot = VGroup(axes, x_label, y_label).scale(0.8).shift(DOWN * 0.5)

        self.play(Create(plot))

        # Define the new functions
        def Lab(x): return Lq * np.cos(x) * np.cos(x - 2*np.pi/3) + Ld * np.sin(x) * np.sin(x - 2*np.pi/3)
        def Lac(x): return Lq * np.cos(x) * np.cos(x + 2*np.pi/3) + Ld * np.sin(x) * np.sin(x + 2*np.pi/3)
        def Lbc(x): return Lq * np.cos(x - 2*np.pi/3) * np.cos(x + 2*np.pi/3) + Ld * np.sin(x - 2*np.pi/3) * np.sin(x + 2*np.pi/3)

        # Create the graphs
        graph_ab = axes.plot(Lab, color=RED)
        graph_ac = axes.plot(Lac, color=GREEN)
        graph_bc = axes.plot(Lbc, color=BLUE)

        # Add labels to the graphs
        label_ab = Text("Lab", color=RED, font_size=24).next_to(graph_ab.points[-1], UR)
        label_ac = Text("Lac", color=GREEN, font_size=24).next_to(graph_ac.points[-1], DR)
        label_bc = Text("Lbc", color=BLUE, font_size=24).next_to(graph_bc.points[-1], UR)

        self.play(Create(graph_ab), Create(graph_ac), Create(graph_bc))
        self.play(Write(label_ab), Write(label_ac), Write(label_bc))

        line_Lq = DashedLine(
            axes.c2p(0, -0.25), axes.c2p(2 * PI, -0.25), 
            color=YELLOW, dash_length=0.1
        )

        line_Ld = DashedLine(
            axes.c2p(0, -0.85), axes.c2p(2 * PI, -0.85), 
            color=YELLOW, dash_length=0.1
        )
        Lq_label = Text("Min", color=YELLOW, font_size=24).next_to(line_Lq, LEFT)
        Ld_label = Text("Max", color=YELLOW, font_size=24).next_to(line_Ld, LEFT)

        self.play(Create(line_Lq), Create(line_Ld), Write(Lq_label), Write(Ld_label))

        self.wait(3)




class EquationsScene(Scene):
    def construct(self):
        # Define the equations
        eq1 = MathTex(r"L_{aa}' = -(L_q - L_d) \sin(2\theta) \omega", color=GREEN)
        eq2 = MathTex(r"L_{bb}' = -(L_q - L_d) \cos(2\theta + \frac{\pi}{6}) \omega", color=GREEN)
        eq3 = MathTex(r"L_{cc}' = (L_q - L_d) \sin(2(\theta + \frac{\pi}{6})) \omega", color=GREEN)
        eq4 = MathTex(r"L_{ab}' = (L_q - L_d) \cos(\frac{\pi}{6} - 2\theta) \omega", color=PINK)
        eq5 = MathTex(r"L_{ac}' = -(L_q - L_d) \cos(2\theta + \frac{\pi}{6}) \omega", color=PINK)
        eq6 = MathTex(r"L_{bc}' = -(L_q - L_d) \sin(2\theta) \omega", color=PINK)

        # Arrange equations in a column
        equationsSelf = VGroup(eq1, eq2, eq3).arrange(DOWN, buff=0.2, aligned_edge=LEFT)
        equationsSelf.move_to(2*UP)

        equationsMutual = VGroup(eq4, eq5, eq6).arrange(DOWN, buff=0.2, aligned_edge=LEFT).next_to(equationsSelf, DOWN, buff=0.2)
        
        # Animate the appearance of equations
        self.play(Write(equationsSelf), run_time=1)
        self.wait(2)
        self.play(Write(equationsMutual), run_time=1)
        self.wait(3)



class SineWaveWithHarmonics(Scene):
    def construct(self):
        # Create axes without numbers
        axes = Axes(
            x_range=[0, 6*np.pi, np.pi/2],
            y_range=[-1.5, 1.5, 0.5],
            axis_config={"color": BLUE, "include_numbers": False},
            tips=False
        )

        # Add labels
        x_label = axes.get_x_axis_label("Angle [rad]")
        y_label = axes.get_y_axis_label("Bemf [V/rad/sec]")

        # Create sine wave
        sine_wave = axes.plot(lambda x: np.sin(x), color=RED)

        # Group all elements
        graph = VGroup(axes, x_label, y_label, sine_wave)

        # Show the graph
        self.play(Create(axes), Write(x_label), Write(y_label))
        self.play(Create(sine_wave))
        self.wait(2)

        # Create function with harmonics
        def harmonic_function(x):
            return (np.sin(x) + 
                    0.1 * np.sin(5*x) - 
                    0.1 * np.sin(7*x) + 
                    0.03 * np.sin(9*x) - 
                    0.03 * np.sin(11*x))

        # Create harmonic wave
        harmonic_wave = axes.plot(harmonic_function, color=GREEN)

        # Fade out the original sine wave and fade in the harmonic wave
        self.play(FadeOut(sine_wave), FadeIn(harmonic_wave), run_time = 1.5)

        self.wait(2)





class TorqueAndFluxEquations(Scene):
    def construct(self):
        # Torque equation
        torque_eq = MathTex(
            r"T = \frac{3}{2} \cdot PP \cdot (\psi \cdot i_q + (L_d - L_q) \cdot i_q \cdot i_d)"
        ).scale(1.2).move_to(2*UP)

        # Legend for torque equation
        legend_torque = VGroup(
            Text("T - Torque [Nm]", slant=ITALIC),
            Text("PP - Pole pairs", slant=ITALIC),
            Text("ψ - Flux linkage [Wb]", slant=ITALIC),
            Text("Ld, Lq - d and q axis inductances [H]", slant=ITALIC),
            Text("id, iq - Current vector projections on the d and q axes [A]", slant=ITALIC)
        ).arrange(DOWN, aligned_edge=LEFT, buff=0.3).scale(0.5)

        # Group torque equation and legend
        torque_group = VGroup(torque_eq, legend_torque).arrange(DOWN, buff=0.8)

        # Show torque equation and legend
        self.play(Write(torque_eq))
        self.play(Write(legend_torque))
        self.wait(5)

        # Fade out torque equation and legend
        self.play(FadeOut(torque_group))

        # Flux linkage equation
        flux_eq = MathTex(
            r"\psi = \frac{K_e}{\frac{3}{2} \cdot PP}"
        ).scale(1.2).move_to(2*UP)

        # Legend for flux linkage equation
        legend_flux = VGroup(
            Text("PP - Pole pairs", slant=ITALIC),
            Text("ψ - Flux linkage [Wb]", slant=ITALIC),
            Text("Ke - BEMF constant [V/rad/sec]", slant=ITALIC)
        ).arrange(DOWN, aligned_edge=LEFT, buff=0.3).scale(0.5)

        # Group flux equation and legend
        flux_group = VGroup(flux_eq, legend_flux).arrange(DOWN, buff=0.8)

        # Show flux linkage equation and legend
        self.play(Write(flux_group))
        self.wait(4)




class SineWaveFrequencyAmplitude(Scene):
    def construct(self):
        # Create axes
        axes = Axes(
            x_range=[0, 2 * np.pi, np.pi / 2],
            y_range=[-4, 4, 1],
            axis_config={"color": BLUE},
            tips=False
        )

        # Add labels to the axes
        x_label = axes.get_x_axis_label("Time [sec]")
        y_label = axes.get_y_axis_label("Amplitude [V]")

        # Initial sine wave (frequency = 1, amplitude = 1)
        sine_wave = always_redraw(
            lambda: axes.plot(lambda x: np.sin(x), color=RED)
        )

        # Show the axes and the initial sine wave
        self.play(Create(axes), Write(x_label), Write(y_label))
        self.play(Create(sine_wave))
        self.wait(1)

        # Increase frequency and amplitude over time
        def update_wave(mob, dt):
            mob.frequency += 0.3*dt  # Gradually increase frequency
            mob.amplitude += 0.3*dt  # Gradually increase amplitude
            mob.become(
                axes.plot(
                    lambda x: mob.amplitude * np.sin(mob.frequency * x),
                    color=RED
                )
            )

        # Add frequency and amplitude attributes to the sine wave
        sine_wave.frequency = 1
        sine_wave.amplitude = 1

        # Apply the updater to the sine wave
        sine_wave.add_updater(update_wave)

        # Let the animation run for a while (e.g., 5 seconds)
        self.wait(10)

        # Remove the updater and stop the animation
        sine_wave.remove_updater(update_wave)






class VoltageCurrentTransferFunction(Scene):
    def construct(self):
        # Transfer function
        transfer_function = MathTex(r"\frac{I}{V} = \frac{1}{Ls + R}")
        transfer_function.to_edge(UP)
        self.play(Write(transfer_function))
        
        # Inject signals text
        inject_text = Text("Inject signals in D and Q axes")
        inject_text.next_to(transfer_function, DOWN)
        self.play(FadeIn(inject_text))
        self.wait(1)
        self.play(FadeOut(inject_text))
        
        # Step response
        axes_step = Axes(
            x_range=[0, 5, 1],
            y_range=[0, 1.2, 0.2],
            axis_config={"color": BLUE},
            x_length=4,
            y_length=3
        )
        axes_step.to_edge(LEFT, buff=1.5).shift(DOWN)

        # Create a perfect step function using Line
        step_voltage = VGroup(
            Line(axes_step.c2p(0, 0), axes_step.c2p(0.5, 0), color=RED),
            Line(axes_step.c2p(0.5, 0), axes_step.c2p(0.5, 1), color=RED),
            Line(axes_step.c2p(0.5, 1), axes_step.c2p(5, 1), color=RED)
        )

        step_current = axes_step.plot(lambda x: 1 - np.exp(-(x-0.5)) if x >= 0.5 else 0, color=GREEN)

        step_label = Text("Step Response", font_size=24).next_to(axes_step, UP)
        step_voltage_label = Text("Voltage", color=RED, font_size=20).next_to(axes_step, LEFT)
        step_current_label = Text("Current", color=GREEN, font_size=20).next_to(step_voltage_label, DOWN)

        self.play(Create(axes_step), Write(step_label))
        self.play(Create(step_voltage))
        self.play(Create(step_current))
        self.play(Write(step_voltage_label), Write(step_current_label))
        
        # Sine wave response
        axes_sine = Axes(
            x_range=[0, 6*np.pi, np.pi/2],
            y_range=[-1.2, 1.2, 0.4],
            axis_config={"color": BLUE},
            x_length=4,
            y_length=3
        )
        axes_sine.to_edge(RIGHT, buff=1.5).shift(DOWN)
        
        sine_voltage = axes_sine.plot(lambda x: np.sin(x), color=RED)
        sine_current = axes_sine.plot(lambda x: 0.8*np.sin(x - np.pi/3), color=GREEN)
        
        sine_label = Text("Sine Response", font_size=24).next_to(axes_sine, UP)
        sine_voltage_label = Text("Voltage", color=RED, font_size=20).next_to(axes_sine, LEFT)
        sine_current_label = Text("Current", color=GREEN, font_size=20).next_to(sine_voltage_label, DOWN)
        
        self.play(Create(axes_sine), Write(sine_label))
        self.play(Create(sine_voltage), Create(sine_current))
        self.play(Write(sine_voltage_label), Write(sine_current_label))
        
        self.wait(2)





if __name__ == "__main__":
    from manim import config

    # Optional: Set output quality here (low or high)
    config.quality = "high_quality"    # Change to "high_quality" for higher resolution
    config.media_dir = os.getcwd()    # Optional: Set output directory

    # Render the scene
    VectorProjections().render()

    # Automatically open the output file
    if platform.system() == 'Windows':
        os.system(f"start {config.output_file}")
        input("Press Enter to exit...")  # Prevents the script from closing immediately. Need in Windows for some reason
    else:
        os.system(f"xdg-open {config.output_file}")
