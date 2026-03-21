# AprilTagAngleCalcs
Angles are confusing

Because cameras project a 3D world onto a 2D sensor, depth ($Z$) is inherently lost. However, by measuring **perspective distortion**, we can recover the exact angle of an object.

<img width="1800" height="1169" alt="Screenshot 2026-03-20 at 10 00 38 PM" src="https://github.com/user-attachments/assets/63f8a15d-1cec-4a4e-9717-fea262ecfbcc" />

## 1. The Core Principle: Perspective Projection
Standard pinhole cameras project objects based on inverse depth. An object's size in pixels ($h$) is inversely proportional to its orthogonal distance from the camera's image plane ($Z$):
$$h \propto \frac{1}{Z}$$

When a flat object (like a drone wing or an AprilTag) is rotated relative to the camera, one side is physically closer to the lens than the other. This depth difference ($\Delta Z$) causes the closer side to appear larger/wider in pixels. By quantifying this visual distortion, we can calculate the angle of rotation, $B$.

## 2. The Math: "The Ratio Trick"
Both pose estimation methods in this visualizer rely on a specific algebraic trick: **The Difference over Sum Ratio**.

If we take the pixel size of the Left side ($h_L$) and the Right side ($h_R$), their ratio elegantly cancels out the camera's focal length and simplifies to the physical depth difference:
$$Ratio = \frac{h_L - h_R}{h_L + h_R} \approx \frac{\frac{1}{Z_L} - \frac{1}{Z_R}}{\frac{1}{Z_L} + \frac{1}{Z_R}} \approx \frac{Z_R - Z_L}{Z_R + Z_L} \approx \frac{\Delta Z}{2 Z_C}$$

Basic trigonometry tells us that the depth difference across an object ($\Delta Z$) is equal to its physical width ($w$) multiplied by the sine of its rotation angle ($B$): 
$$\Delta Z = w \cdot \sin(B)$$

Substituting this back into the ratio gives us our master equation for pose estimation:
$$\sin(B) \approx \left(\frac{2 \cdot Z_C}{w}\right) \cdot \left(\frac{h_L - h_R}{h_L + h_R}\right)$$

### Method A: Multi-Tag Spacing
Instead of looking at a single tag, we can look at the pixel distance between three separate tags.
* Let **$\Delta_1$** and **$\Delta_2$** be the pixel spacing between the Left-Center and Center-Right tags.
* Let **$W$** be the physical wingspan (distance between the outer tags).

$$\sin(B) \approx \left(\frac{2 \cdot Z_C}{W}\right) \cdot \left(\frac{\Delta_1 - \Delta_2}{\Delta_1 + \Delta_2}\right)$$
*(Note: Method A is generally more accurate than Method B because it spans a larger physical area, minimizing the impact of pixel-rounding errors).*

### Method B: Single-Tag Distortion
Uses the left and right pixel edges (**$h_L$** and **$h_R$**) of a single AprilTag with a physical width of **$w$**.
$$\sin(B) \approx \left(\frac{2 \cdot Z_C}{w}\right) \cdot \left(\frac{h_L - h_R}{h_L + h_R}\right)$$

## 3. Global Normal Recovery
Methods A and B calculate **Angle $B$**—the angle of the object *relative to the camera's optical axis*. 
To find the true global orientation (compass heading) of the object in the world, we subtract the relative angle from the camera's fixed gaze angle:
$$\text{Global Normal} = \text{Camera Optical Axis} - B$$

## 4. The Translational Distortion Paradox
**Question:** *If the object moves to the far edge of the camera's field of view, it gets further away from the lens. Won't the edge furthest from the camera appear smaller, generating a fake rotation angle even if the object is perfectly parallel?*

**Answer:** No. The algorithm is **100% immune to translational distortion**.

Standard cameras (and the Intrinsic Matrix used in this simulation) project onto a **flat 2D sensor**, resulting in **Rectilinear Projection**. In rectilinear projection, an object's size in pixels is determined *only* by its orthogonal depth ($Z$) to the sensor plane, not its true Euclidean distance ($D = \sqrt{X^2+Y^2+Z^2}$) to the lens.
$$h_{pixels} = f \cdot \frac{H_{physical}}{Z}$$

Even though an object at the edge of the frame is physically further from the lens, the light rays hit the flat sensor at a steeper angle. This "stretches" the projection across more pixels, which **perfectly cancels out** the shrinking effect of the increased distance.

Furthermore, if the object *is* rotated, any offset $X_C$ in the camera frame entirely cancels itself out during the Difference-over-Sum fraction: $\frac{\Delta_1 - \Delta_2}{\Delta_1 + \Delta_2}$. 

Regardless of where the object is on the screen, the ratio of visual distortion remains a mathematically pure representation of the depth difference ($\Delta Z$).

***

### Controls
* **W, A, S, D** or **Mouse Drag**: Move the drone in 3D space.
* **Q, E** or **Scroll Wheel**: Rotate the drone's yaw.
* **ESC**: Quit simulation.

<img width="963" height="724" alt="Screenshot 2026-03-18 at 11 21 43 PM" src="https://github.com/user-attachments/assets/9ad7e756-66e2-4cae-b7f5-8f12b08eb726" />
<img width="969" height="929" alt="Screenshot 2026-03-18 at 11 21 29 PM" src="https://github.com/user-attachments/assets/9bbe32e2-9f9d-4983-9141-1a9c29b15d8c" />
