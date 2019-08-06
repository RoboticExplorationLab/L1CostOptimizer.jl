using Colors: RGBA, RGB
using CoordinateTransformations
using FileIO
using GeometryTypes:
    GeometryTypes, HyperRectangle, Vec, Point,
    HomogenousMesh, SignedDistanceField, HyperSphere, GLUVMesh, Pyramid
using JLD
using LinearAlgebra
using MeshCat
using MeshIO

function define_animation_parameters()
    # Defines the necessary parameters for animating a trajectory.
    filename = "constrained_nonlinear_dynamics"
    parameters = Dict("filename" => filename, # filename of the trajectory file.
                  "length_scale" => 1e-6, # scaling of the lengths of the problem.
                  "delta_scale" => 1e2, # scaling of the delta between the 2 satellites.
                  "cubesat_dim_x" => 3e-1, # length of the cubesat along X.
                  "cubesat_dim_y" => 1e-1, # length of the cubesat along Y.
                  "cubesat_dim_z" => 1e-1, # length of the cubesat along Z.
                  "pyramid_point" => Point(0.0, 0.0, 0.0), # Center of the pyramid indicating controls.
                  "pyramid_length" => 75.0, # Length of the pyramid indicating controls.
                  "pyramid_width" => 4e-2, # Width of the pyramid indicating controls.
                  "pyramid_rot_x" => LinearMap(AngleAxis(pi/2, 0.0, 1.0, 0.0)), # Rotation of the pyramid to indicate forces along X.
                  "pyramid_rot_y" => LinearMap(AngleAxis(-pi/2, 1.0, 0.0, 0.0)), # Rotation of the pyramid to indicate forces along Y.
                  "pyramid_rot_z" => LinearMap(AngleAxis(0.0, 1.0, 0.0, 0.0)), # Rotation of the pyramid to indicate forces along Z.
                  "sphere_radius" => 3e-2, # Radius of the sphere indicating ego node points.
                  "camera_translation" => Translation(-1.0, 1.0, 1.0), # translation of the camera wrt the target satellite.
                  "camera_rotation" => compose(LinearMap(AngleAxis(0.1*pi, 1.0, 0.0, 1.0)), LinearMap(AngleAxis(0.6*pi, 0.0, 0.0, 1.0))), # rotation of the camera wrt the target satellite.
                  "camera_zoom" => 2.8, # zoom of the camera.
                  )
    return parameters
end

function spacecraft_transformation(x, parameters)
    # Compute the rotations and translations of the spacecrafts.
    length_scale = parameters["length_scale"]
    delta_scale = parameters["delta_scale"]
    r_target = x[4:6]
    rd_target = x[10:12]
    r_ego = r_target - delta_scale*x[1:3]
    target_translation = r_target*length_scale
    ego_translation = r_ego*length_scale

    # define the axis of the C-W frame as deined in
    # R. Wiltshire and W. Clohessy,
    # “Terminal guidance system for satellite rendezvous”.
    ey_cw = r_target / norm(r_target)
    ez_cw = cross(ey_cw, rd_target)
    ez_cw = ez_cw / norm(ez_cw)
    ex_cw = cross(ey_cw, ez_cw)
    R_wc = [ex_cw ey_cw ez_cw]
    target_rotation = LinearMap(RotMatrix(R_wc...))
    target_translation = Translation(target_translation...)
    ego_translation = Translation(ego_translation...)
    return target_translation, target_rotation, ego_translation
end

function load_data(parameters)
    # Load data
    filename = parameters["filename"]
    T = load("animation/trajectory/" * filename * ".jld", "T")
    X = load("animation/trajectory/" * filename * ".jld", "X")
    Y = load("animation/trajectory/" * filename * ".jld", "Y")
    U = load("animation/trajectory/" * filename * ".jld", "U")
    N = length(T)
    n = size(X)[1]
    m = size(U)[1]
    return T, X, Y, U, N, n, m
end

function visualizer(parameters)
    # Visualizes the trajectories obtained using the L1 cost optimizer using Meshcat.
    length_scale = parameters["length_scale"]
    cubesat_dim_x = parameters["cubesat_dim_x"]
    cubesat_dim_y = parameters["cubesat_dim_y"]
    cubesat_dim_z = parameters["cubesat_dim_z"]
    pyramid_point = parameters["pyramid_point"]
    pyramid_length = parameters["pyramid_length"]
    pyramid_width = parameters["pyramid_width"]
    pyramid_rot_x = parameters["pyramid_rot_x"]
    pyramid_rot_y = parameters["pyramid_rot_y"]
    pyramid_rot_z = parameters["pyramid_rot_z"]
    sphere_radius = parameters["sphere_radius"]
    camera_translation = parameters["camera_translation"]
    camera_rotation = parameters["camera_rotation"]
    camera_zoom = parameters["camera_zoom"]

    # Load Data
    T, X, Y, U, N, n, m = load_data(parameters)

    # Open visualizer
    vis = Visualizer()
    open(vis)

    # Plot Earth in MeshCat
    earth_image = PngImage("animation/image/earth.png")
    earth_texture = Texture(image=earth_image)
    earth_material = MeshLambertMaterial(map=earth_texture)
    earth = HyperSphere(Point(0.0,0.0,0.0), 6.7310*1e6*length_scale)
    setobject!(vis["earth"], earth, earth_material)
    settransform!(vis["earth"], compose(LinearMap(AngleAxis(-0.8*pi, 0, 0, 1)),
        LinearMap(AngleAxis(pi/2, 1, 0, 0)), ))

    # Plot Target Satellite
    target_cubesat_image = PngImage("animation/image/white_solar_panel.png")
    target_cubesat_texture = Texture(image=target_cubesat_image)
    target_cubesat_material = MeshLambertMaterial(map=target_cubesat_texture)
    target_cubesat = HyperRectangle(-Vec(cubesat_dim_x, cubesat_dim_y, cubesat_dim_z)./2,
        Vec(cubesat_dim_x, cubesat_dim_y, cubesat_dim_z))
    setobject!(vis["target_cubesat"], target_cubesat, target_cubesat_material)

    # Plot Ego Satellite
    ego_cubesat_image = PngImage("animation/image/orange_solar_panel.png")
    ego_cubesat_texture = Texture(image=ego_cubesat_image)
    ego_cubesat_material = MeshLambertMaterial(map=ego_cubesat_texture)
    ego_cubesat = HyperRectangle(-Vec(-cubesat_dim_x, cubesat_dim_y, cubesat_dim_z)./2,
        Vec(cubesat_dim_x, cubesat_dim_y, cubesat_dim_z))
    setobject!(vis["ego_cubesat"], ego_cubesat, ego_cubesat_material)

    # Material
    light_green_material = MeshPhongMaterial(color=RGBA(0, 1, 0, 3e-1))
    green_material = MeshPhongMaterial(color=RGBA(0, 1, 0, 9e-1))
    yellow_material = MeshPhongMaterial(color=RGBA(1, 1, 0, 7e-1))
    red_material = MeshPhongMaterial(color=RGBA(1, 0, 0, 9e-1))

    # Plot Trajectory
    anim = MeshCat.Animation()
    for i=1:N
        # Compute spacecraft transformation
        target_translation, target_rotation, ego_translation =
            spacecraft_transformation(X[:,i], parameters)
        # Set the poses of the two satellites as well as the camera.
        MeshCat.atframe(anim, vis, i) do frame
            settransform!(frame["target_cubesat"], compose(target_translation, target_rotation))
            settransform!(frame["ego_cubesat"], compose(ego_translation, target_rotation))
            setprop!(frame["/Cameras/default/rotated/<object>"], "zoom", camera_zoom)
            camera_transformation = compose(compose(compose(
                target_translation,
                target_rotation),
                camera_translation),
                camera_rotation)
            settransform!(frame["/Cameras/default"], camera_transformation)
        end
        # Plot the forces vector along the ego satellite's trajectory.
        if i < N
            # Build objects.
            sphere = HyperSphere(Point(0.0,0.0,0.0), sphere_radius)
            pyramid_x = Pyramid(pyramid_point, pyramid_length*U[1,i], pyramid_width)
            pyramid_y = Pyramid(pyramid_point, pyramid_length*U[2,i], pyramid_width)
            pyramid_z = Pyramid(pyramid_point, pyramid_length*U[3,i], pyramid_width)
            # Add them to the visualizer
            setobject!(vis["traj"]["t_sph$i"], sphere, light_green_material)
            setobject!(vis["traj"]["t_x$i"], pyramid_x, yellow_material)
            setobject!(vis["traj"]["t_y$i"], pyramid_y, red_material)
            setobject!(vis["traj"]["t_z$i"], pyramid_z, green_material)
            # Define their poses
            settransform!(vis["traj"]["t_sph$i"], ego_translation)
            settransform!(vis["traj"]["t_x$i"], compose(compose(ego_translation, target_rotation), pyramid_rot_x))
            settransform!(vis["traj"]["t_y$i"], compose(compose(ego_translation, target_rotation), pyramid_rot_y))
            settransform!(vis["traj"]["t_z$i"], compose(compose(ego_translation, target_rotation), pyramid_rot_z))
        end
    end
    MeshCat.setanimation!(vis,anim)
end

# parameters = define_animation_parameters()
# parameters["filename"] = "constrained_nonlinear_dynamics"
# visualizer(parameters)

# Saving as a video.
MeshCat.convert_frames_to_video(
    "/home/simon/research/l1_optimization/L1CostOptimizer.jl/animation/meshcat_1565050100592.tar", overwrite=true)

# To convert the still frames into a video, extract the `.tar` file and run:
# ffmpeg -r 60 -i %07d.png \
# 	 -vcodec libx264 \
# 	 -preset slow \
# 	 -crf 18 \
# 	 output.mp4
