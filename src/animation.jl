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
                  "cubesat_dims" => [3e-1, 1e-1, 1e-1], # dimensions of the cubesat along X.
                  "pyramid_point" => Point(0.0, 0.0, 0.0), # Center of the pyramid indicating controls.
                  "pyramid_width" => 4e-2, # Width of the pyramid indicating controls.
                  "pyramid_rot_x" => LinearMap(AngleAxis(pi/2, 0.0, 1.0, 0.0)), # Rotation of the pyramid to indicate forces along X.
                  "pyramid_rot_y" => LinearMap(AngleAxis(-pi/2, 1.0, 0.0, 0.0)), # Rotation of the pyramid to indicate forces along Y.
                  "pyramid_rot_z" => LinearMap(AngleAxis(0.0, 1.0, 0.0, 0.0)), # Rotation of the pyramid to indicate forces along Z.
                  "sphere_radius" => 3e-2, # Radius of the sphere indicating ego node points.
                  "camera_translation" => Translation(-1.0, 1.0, 1.0), # translation of the camera wrt the target satellite.
                  "camera_rotation" => compose(LinearMap(AngleAxis(0.13*pi, 1.0, 0.0, 1.0)), LinearMap(AngleAxis(0.55*pi, 0.0, 0.0, 1.0))), # rotation of the camera wrt the target satellite.
                  "camera_zoom" => 4.8, # zoom of the camera.
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
    cubesat_dims = parameters["cubesat_dims"]
    pyramid_point = parameters["pyramid_point"]
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
    # extend control sequence for visualization purpose
    U_extended = zeros(m,N)
    U_extended[:,1:N-1] = U
    # U_extended[:,N] = U[:,N-1]
    # Determine the right length for the pyramid
    u_max = norm(U, Inf)

    max_lengths = cubesat_dims.*0.95
    println("cubesat_dims./maximum(U, dims=2) = ",  cubesat_dims./maximum(U, dims=2))
    println("maximum(U, dims=2) = ",  maximum(U, dims=2))
    println("u_max = ",  u_max)
    println("max_lengths = ",  max_lengths)

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
    target_cubesat = HyperRectangle(-Vec(cubesat_dims...)./2, Vec(cubesat_dims...))
    setobject!(vis["target_cubesat"], target_cubesat, target_cubesat_material)

    # Plot Ego Satellite
    ego_cubesat_image = PngImage("animation/image/orange_solar_panel.png")
    ego_cubesat_texture = Texture(image=ego_cubesat_image)
    ego_cubesat_material = MeshLambertMaterial(map=ego_cubesat_texture)
    ego_cubesat = HyperRectangle(-Vec(-cubesat_dims[1], cubesat_dims[2], cubesat_dims[3])./2,
        Vec(cubesat_dims...))
    setobject!(vis["ego_cubesat"], ego_cubesat, ego_cubesat_material)

    # Material
    light_green_material = MeshPhongMaterial(color=RGBA(0, 1, 0, 3e-1))
    green_material = MeshPhongMaterial(color=RGBA(0, 1, 0, 10e-1))
    yellow_material = MeshPhongMaterial(color=RGBA(1, 1, 0, 10e-1))
    red_material = MeshPhongMaterial(color=RGBA(1, 0, 0, 10e-1))

    # Plot Trajectory
    anim = MeshCat.Animation()
    for i=1:N
        # Compute spacecraft transformation
        target_translation, target_rotation, ego_translation =
            spacecraft_transformation(X[:,i], parameters)
        # Useful translation to put the pyramids at the center of the ego cubesat.
        ego_shift = Translation([cubesat_dims[1], 0, 0]...)
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
        # Build objects.
        pyramid_x = Pyramid(pyramid_point, max_lengths[1], pyramid_width)
        pyramid_y = Pyramid(pyramid_point, max_lengths[2], pyramid_width)
        pyramid_z = Pyramid(pyramid_point, max_lengths[3], pyramid_width)
        # Add them to the visualizer
        setobject!(vis["pyramid_x_pos"], pyramid_x, yellow_material)
        setobject!(vis["pyramid_y_pos"], pyramid_y, red_material)
        setobject!(vis["pyramid_z_pos"], pyramid_z, green_material)
        setobject!(vis["pyramid_x_neg"], pyramid_x, yellow_material)
        setobject!(vis["pyramid_y_neg"], pyramid_y, red_material)
        setobject!(vis["pyramid_z_neg"], pyramid_z, green_material)
        pyramid_shift_pos, pyramid_rot_pos = pyramid_pos_placement(U_extended[:,i], u_max, cubesat_dims, max_lengths)
        pyramid_shift_neg, pyramid_rot_neg = pyramid_neg_placement(U_extended[:,i], u_max, cubesat_dims, max_lengths)

        # Define their poses
        MeshCat.atframe(anim, vis, i) do frame
            settransform!(frame["pyramid_x_pos"], compose(compose(compose(compose(ego_translation, target_rotation), ego_shift), pyramid_shift_pos[1]), pyramid_rot_pos[1]))
            settransform!(frame["pyramid_y_pos"], compose(compose(compose(compose(ego_translation, target_rotation), ego_shift), pyramid_shift_pos[2]), pyramid_rot_pos[2]))
            settransform!(frame["pyramid_z_pos"], compose(compose(compose(compose(ego_translation, target_rotation), ego_shift), pyramid_shift_pos[3]), pyramid_rot_pos[3]))
            settransform!(frame["pyramid_x_neg"], compose(compose(compose(compose(ego_translation, target_rotation), ego_shift), pyramid_shift_neg[1]), pyramid_rot_neg[1]))
            settransform!(frame["pyramid_y_neg"], compose(compose(compose(compose(ego_translation, target_rotation), ego_shift), pyramid_shift_neg[2]), pyramid_rot_neg[2]))
            settransform!(frame["pyramid_z_neg"], compose(compose(compose(compose(ego_translation, target_rotation), ego_shift), pyramid_shift_neg[3]), pyramid_rot_neg[3]))
        end
    end
    MeshCat.setanimation!(vis,anim)
end


function pyramid_pos_placement(u, u_max, cubesat_dims, max_lengths)
    pyramid_shift = []
    pyramid_rot = []
    trans_x = -(1 - max(u[1],0)/u_max)*(max_lengths[1])+cubesat_dims[1]/2
    push!(pyramid_rot, LinearMap(AngleAxis(pi/2, 0.0, 1.0, 0.0)))
    trans_y = (1 - max(u[2],0)/u_max)*(max_lengths[2])-cubesat_dims[2]/2
    push!(pyramid_rot, LinearMap(AngleAxis(pi/2, 1.0, 0.0, 0.0)))
    trans_z = (1 - max(u[3],0)/u_max)*(max_lengths[3])-cubesat_dims[3]/2
    push!(pyramid_rot, LinearMap(AngleAxis(pi, 1.0, 0.0, 0.0)))

    pyramid_shift = [Translation([trans_x, 0, 0]...),
        Translation([0, trans_y, 0]...),
        Translation([0, 0, trans_z]...)]
    return pyramid_shift, pyramid_rot
end


function pyramid_neg_placement(u, u_max, cubesat_dims, max_lengths)
    pyramid_shift = []
    pyramid_rot = []
    trans_x = (1 - max(-u[1],0)/u_max)*(max_lengths[1])-cubesat_dims[1]/2
    push!(pyramid_rot, LinearMap(AngleAxis(-pi/2, 0.0, 1.0, 0.0)))
    trans_y = -(1 - max(-u[2],0)/u_max)*(max_lengths[2])+cubesat_dims[2]/2
    push!(pyramid_rot, LinearMap(AngleAxis(-pi/2, 1.0, 0.0, 0.0)))
    trans_z = -(1 - max(-u[3],0)/u_max)*(max_lengths[3])+cubesat_dims[3]/2
    push!(pyramid_rot, LinearMap(AngleAxis(0.0, 1.0, 0.0, 0.0)))

    pyramid_shift = [Translation([trans_x, 0, 0]...),
        Translation([0, trans_y, 0]...),
        Translation([0, 0, trans_z]...)]
    return pyramid_shift, pyramid_rot
end

parameters = define_animation_parameters()
parameters["filename"] = "quadratic_unconstrained_nonlinear_dynamics"
# parameters["filename"] = "constrained_nonlinear_dynamics"
visualizer(parameters)
# # Saving as a video.
MeshCat.convert_frames_to_video(
    "/home/simon/research/l1_optimization/L1CostOptimizer.jl/animation/meshcat_1565253223544.tar", overwrite=true)
