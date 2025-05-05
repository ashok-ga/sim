# MuJoCo Joint Parameter Calculation

A **friendly, step-by-step** guide to compute the core joint parameters needed for realistic MuJoCo simulations:

- **Armature**: Rotational inertia seen at the joint.  
- **Friction Loss**: Damping coefficient that resists motion.  
- **Actuator Range**: Maximum torque or force your motor and gearbox can apply.  
- **Joint Limits**: The physical angle (or distance) you allow the joint to travel.  
- **PD Gains (kp, kv)**: Proportional and derivative gains for MuJoCo’s built-in position actuator.

---

## 📁 Project Layout

```
/your-mujoco-project
│
├── models/
│   └── humanoid.xml        # Your MuJoCo XML using <default> blocks
│
├── docs/
│   └── Calculating_MuJoCo_Joint_Parameters.md  # Full derivation guide
│
└── README.md              # (this file) quick reference
```

---

## 🔍 Key Terms & Sources

| Parameter        | What it means                                 | Where to get it                       |
|------------------|-----------------------------------------------|---------------------------------------|
| **armature**     | Rotational inertia (kg·m²)                     | CAD inertia tool                      |
| **frictionloss** | Viscous damping (N·m·s/rad)                    | No-load torque vs. speed test         |
| **actuator range** | Torque limits (continuous & peak) (N·m)      | Motor & gearbox datasheet             |
| **joint limits** | Mechanical angle or distance bounds           | CAD stops or physical measurement     |
| **kp**, **kv**   | PD gains for smooth position control          | Desired rise time + `armature`        |

---

## 🧮 How to Calculate Each Parameter

### 1. Armature (Rotational Inertia)
1. Export the **inertia tensor** $I_{COM}$ from your CAD model (about the link’s COM).  
2. Define the **joint axis** as a unit vector $\mathbf{a}$.  
3. Compute motor-side inertia:
   $$
     I_{motor} = \mathbf{a}^T \, I_{COM} \, \mathbf{a}
   $$
4. If you have a gearbox with ratio $r$ (output:motor):
   $$
     I_{joint} = I_{motor} 	imes r^2.
   $$

### 2. Friction Loss (Damping)
1. Run the motor+gear **unloaded** at a constant speed $\omega$ (rad/s).  
2. Measure steady torque $T_f$ needed to hold speed.  
3. Calculate:
   $$
     b = rac{T_f}{\omega}\quadigl(	ext{N·m·s/rad}igr).
   $$

### 3. Actuator Range (Torque Limits)
- **Continuous torque** $	au_{rated}$ → set `actuatorfrcrange="-\tau_{rated} \tau_{rated}"`.  
- **Peak torque** $	au_{peak}$ → use for short bursts: `"-\tau_{peak} \tau_{peak}"`.

### 4. Joint Limits (Range)
In your MuJoCo XML:
```xml
<joint range="lower upper"/>  <!-- radians or meters -->
```
Derive from your CAD assembly stops or by physically moving the joint and reading your encoder.

### 5. PD Gains (kp, kv)
Design for a **rise time** $t_r$ and **critical damping** (ζ=1):
1. Natural frequency:
   $$
     \omega_n = rac{1.8}{t_r}.
   $$
2. Compute gains using your computed $I_{joint}$:
   $$
     k_p = I_{joint} \, \omega_n^2,
     \quad
     k_v = 2 \, I_{joint} \, \omega_n.
   $$

**Example:**
- $I_{joint}=0.0435$ kg·m², desired $t_r=0.2$ s → $\omega_n=9$ rad/s  
- $k_p=0.0435\times9^2\approx3.5$,  $k_v=2\times0.0435\times9\approx0.78$  

---

## ⚙️ Quick MuJoCo Integration

Insert into your `<default>` block:
```xml
<default class="your_joint_class">
  <joint
    armature="I_joint"
    frictionloss="b"
    actuatorfrcrange="-tau tau"
    range="lower upper"
  />
  <position kp="kp" kv="kv"/>
</default>
```
Replace `I_joint`, `b`, `tau`, `lower`, `upper`, `kp`, and `kv` with your calculated numbers.

---

## 🛠️ Tips & Best Practices
- **Inertia fidelity:** export inertia from a high-resolution mesh in CAD.  
- **Smooth friction measurement:** average torque readings over time.  
- **Gain tuning:** start with these formulas, then fine-tune in simulation to avoid overshoot or sluggishness.

---

> _This README is your quick-reference cheat-sheet. For full derivations and examples, see `docs/Calculating_MuJoCo_Joint_Parameters.md`._