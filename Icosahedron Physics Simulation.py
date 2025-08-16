import pyvista as pv
import numpy as np
import time
from scipy.spatial.transform import Rotation as R
from numba import jit

print("--- Final Icosahedron Physics Simulation & Renderer ---")
print("--- Version: 30 seconds, 4x Speed ---")

# --- 1. SIMULATION PARAMETERS ---
class Config:
    # --- FINAL TWEAKS ---
    DURATION = 30.0  # seconds (Twice as long)
    FPS = 240        # frames per second (4x faster playback)
    # ---
    
    DT = 1.0 / FPS
    NUM_FRAMES = int(DURATION * FPS)
    
    # Perpetual Motion Physics Settings
    GRAVITY = np.array([0.0, 0.0, 0.0])
    RESTITUTION = 1.0
    FRICTION = 0.0
    
    SPHERE_RADIUS = 0.5
    SPHERE_INITIAL_POS = np.array([1.0, 3.0, 0.0])
    SPHERE_INITIAL_VEL = np.array([-2.0, 1.0, -1.5])
    
    ICOSAHEDRON_RADIUS = 5.0
    ROTATION_AXIS = np.array([0.1, 1.0, 0.2])
    ROTATION_AXIS /= np.linalg.norm(ROTATION_AXIS)
    ANGULAR_VELOCITY = 0.4

# --- 2. GENERATE THE BASE GEOMETRY ---
source_icosahedron = pv.Icosahedron(radius=Config.ICOSAHEDRON_RADIUS)
initial_vertices = source_icosahedron.points.astype(np.float64)
initial_faces = source_icosahedron.faces.reshape(-1, 4)[:, 1:]

print(f"Generated a perfect Icosahedron with {len(initial_faces)} faces.")

# --- 3. THE HIGH-SPEED, JIT-COMPILED PHYSICS ENGINE ---
@jit(nopython=True)
def run_physics_simulation(num_frames, dt, sphere_pos, sphere_vel, cage_verts, cage_faces,
                           gravity, restitution, friction, sphere_radius, rotation_axis, angular_velocity):
    
    sphere_pos_history = np.zeros((num_frames, 3))
    cage_verts_history = np.zeros((num_frames, len(cage_verts), 3))

    angle = angular_velocity * dt
    c, s = np.cos(angle), np.sin(angle)
    ax = rotation_axis
    rot_mat = np.array([
        [c + ax[0]**2*(1-c), ax[0]*ax[1]*(1-c) - ax[2]*s, ax[0]*ax[2]*(1-c) + ax[1]*s],
        [ax[1]*ax[0]*(1-c) + ax[2]*s, c + ax[1]**2*(1-c), ax[1]*ax[2]*(1-c) - ax[0]*s],
        [ax[2]*ax[0]*(1-c) - ax[1]*s, ax[2]*ax[1]*(1-c) + ax[0]*s, c + ax[2]**2*(1-c)]
    ])
    current_verts = cage_verts.copy()

    for frame in range(num_frames):
        sphere_vel += gravity * dt
        sphere_pos += sphere_vel * dt
        current_verts = np.dot(current_verts, rot_mat.T)
        
        for i in range(len(cage_faces)):
            face_indices = cage_faces[i]
            v0, v1, v2 = current_verts[face_indices[0]], current_verts[face_indices[1]], current_verts[face_indices[2]]
            
            normal = np.cross(v2 - v0, v1 - v0)
            normal_len = np.linalg.norm(normal)
            if normal_len > 1e-6:
                normal /= normal_len
                distance = np.dot(sphere_pos - v0, normal)
                if distance < sphere_radius:
                    penetration = sphere_radius - distance
                    sphere_pos += normal * penetration
                    vel_dot_normal = np.dot(sphere_vel, normal)
                    if vel_dot_normal < 0:
                        normal_vel = normal * vel_dot_normal
                        tangent_vel = sphere_vel - normal_vel
                        tangent_vel *= (1.0 - friction)
                        sphere_vel = tangent_vel - normal_vel * restitution
        
        sphere_pos_history[frame] = sphere_pos
        cage_verts_history[frame] = current_verts

    return sphere_pos_history, cage_verts_history

# --- 4. PREPARE AND RUN THE SIMULATION ---
print(f"Simulating {Config.DURATION} seconds of physics at {Config.FPS} FPS...")
print(f"This will generate a total of {Config.NUM_FRAMES} frames. Please be patient...")
start_time = time.time()
sphere_positions, cage_vertices_history = run_physics_simulation(
    Config.NUM_FRAMES, Config.DT, 
    Config.SPHERE_INITIAL_POS.copy(), 
    Config.SPHERE_INITIAL_VEL.copy(),
    initial_vertices,
    initial_faces,
    Config.GRAVITY,
    Config.RESTITUTION,
    Config.FRICTION,
    Config.SPHERE_RADIUS,
    Config.ROTATION_AXIS,
    Config.ANGULAR_VELOCITY
)
end_time = time.time()
print(f"Simulation complete in {end_time - start_time:.2f} seconds.")

# --- 5. RENDER THE ANIMATION TO A GIF ---
plotter = pv.Plotter(off_screen=True)
plotter.background_color = 'black'

sphere_mesh = pv.Sphere(radius=Config.SPHERE_RADIUS, center=sphere_positions[0])
plotter.add_mesh(sphere_mesh, color='red', name='sphere')

cage_mesh = source_icosahedron.copy()
plotter.add_mesh(cage_mesh, style='wireframe', color='#00ffff', line_width=3, name='cage')

output_filename = "icosahedron_simulation_long_fast.gif"
print(f"Rendering animation to {output_filename}...")
plotter.open_gif(output_filename, fps=Config.FPS)
plotter.camera_position = 'iso'
plotter.camera.zoom(1.6)

for frame in range(Config.NUM_FRAMES):
    sphere_mesh.points = pv.Sphere(radius=Config.SPHERE_RADIUS, center=sphere_positions[frame]).points
    cage_mesh.points = cage_vertices_history[frame]
    plotter.write_frame()

plotter.close()
print("--- Rendering complete! ---")
print(f"Please open the file '{output_filename}' to view the final animation.")
