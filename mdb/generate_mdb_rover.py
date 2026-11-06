import yamcs.pymdb as MDB

from _mdb_cli import write_image_buckets, write_mdb
from _omnilrs import load_cameras, load_naming, load_streaming_fields, load_yamcs_tmtc, omnilrs_root, parse_args

args = parse_args("Generate the rover MDB XML from the OmniLRS robot definition.")
root = omnilrs_root(args.omnilrs_root)
naming = load_naming(root)
cameras = load_cameras(root, args.robot)
yamcs_tmtc = load_yamcs_tmtc(root, args.controller)
streaming_fields = load_streaming_fields(root, args.robot)

rover = MDB.System("Rover")

# Create subsystems for organizing telemetry and telecommands (TMTC)
system_subsystem = MDB.Subsystem(rover, "system", short_description="System-level TMTC")
power_subsystem = MDB.Subsystem(rover, "power", short_description="Power management TMTC")
motor_subsystem = MDB.Subsystem(rover, "motor", short_description="Motor and drive TMTC")
camera_subsystem = MDB.Subsystem(rover, "camera", short_description="Camera control TMTC")
# (camera image metadata space systems are created per robot camera, see "Camera telemetry" below)

###################
#### Telemetry ####
###################

# On Board Computer
obc_state = MDB.EnumeratedParameter(
    system=rover,
    name="obc_state",
    choices=[
        (0, "OFF"),
        (1, "BOOT"),
        (2, "IDLE"),
        (3, "CAMERA"),
        (4, "MOTOR"),
        (5, "SAFE"),
        (6, "ERROR"),
    ],
    data_source=MDB.DataSource.LOCAL,
    short_description="On-Board Computer state",
)

obc_cpu_usage = MDB.IntegerParameter(
    system=rover,
    name="obc_cpu_usage",
    signed=False,
    bits=8,
    minimum=0,
    maximum=100,
    data_source=MDB.DataSource.LOCAL,
    short_description="On-Board Computer CPU usage",
    units="%",
)

obc_ram_usage = MDB.IntegerParameter(
    system=rover,
    name="obc_ram_usage",
    signed=False,
    bits=8,
    minimum=0,
    maximum=100,
    data_source=MDB.DataSource.LOCAL,
    short_description="On-Board Computer RAM usage",
    units="%",
)

obc_disk_usage = MDB.IntegerParameter(
    system=rover,
    name="obc_disk_usage",
    signed=False,
    bits=8,
    minimum=0,
    maximum=100,
    data_source=MDB.DataSource.LOCAL,
    short_description="On-Board Computer disk usage",
    units="%",
)

obc_uptime = MDB.FloatParameter(
    system=rover,
    name="obc_uptime",
    bits=32,
    minimum=0.0,
    data_source=MDB.DataSource.LOCAL,
    short_description="On-Board Computer uptime",
    units="s",
)

# Mission Status
go_nogo = MDB.EnumeratedParameter(
    system=rover,
    name="go_nogo",
    choices=[
        (0, "NOGO"),
        (1, "GO"),
    ],
    data_source=MDB.DataSource.LOCAL,
    short_description="Current GO/NOGO mission status",
)

solar_panel_state = MDB.EnumeratedParameter(
    system=rover,
    name="solar_panel_state",
    choices=[
        (0, "STOWED"),
        (1, "DEPLOYED"),
    ],
    data_source=MDB.DataSource.LOCAL,
    short_description="Deployment state of the solar panel",
)


# Ground Truth Pose: position (x, y, z) and orientation (w, x, y, z)
pose_ground_truth = MDB.AggregateParameter(
    system=rover,
    name="pose_ground_truth",
    members=[
        MDB.AggregateMember(
            name="position",
            members=[
                MDB.FloatMember(
                    name="x",
                    bits=32,
                    units="m",
                    short_description="X coordinate",
                ),
                MDB.FloatMember(
                    name="y",
                    bits=32,
                    units="m",
                    short_description="Y coordinate",
                ),
                MDB.FloatMember(
                    name="z",
                    bits=32,
                    units="m",
                    short_description="Z coordinate",
                ),
            ],
            short_description="3D position in Cartesian coordinates",
        ),
        MDB.AggregateMember(
            name="orientation",
            members=[
                MDB.FloatMember(
                    name="w",
                    bits=32,
                    minimum=-1.0,
                    maximum=1.0,
                    short_description="Quaternion W (scalar component)",
                ),
                MDB.FloatMember(
                    name="x",
                    bits=32,
                    minimum=-1.0,
                    maximum=1.0,
                    short_description="Quaternion X component",
                ),
                MDB.FloatMember(
                    name="y",
                    bits=32,
                    minimum=-1.0,
                    maximum=1.0,
                    short_description="Quaternion Y component",
                ),
                MDB.FloatMember(
                    name="z",
                    bits=32,
                    minimum=-1.0,
                    maximum=1.0,
                    short_description="Quaternion Z component",
                ),
            ],
            short_description="3D orientation as quaternion (w, x, y, z)",
        ),
    ],
    data_source=MDB.DataSource.LOCAL,
    short_description="Ground truth pose with position and orientation",
)


# IMU
orientation = MDB.AggregateParameter(
    system=rover,
    name="imu_orientation",
    members=[
        MDB.FloatMember(
            name="roll",
            bits=32,
            minimum=-180.0,
            maximum=180.0,
            units="deg",
        ),
        MDB.FloatMember(
            name="pitch",
            bits=32,
            minimum=-180.0,
            maximum=180.0,
            units="deg",
        ),
        MDB.FloatMember(
            name="yaw",
            bits=32,
            minimum=-180.0,
            maximum=180.0,
            units="deg",
        ),
    ],
    data_source=MDB.DataSource.LOCAL,
    short_description="Orientation data with roll, pitch, and yaw",
)

accelerometer = MDB.AggregateParameter(
    system=rover,
    name="imu_accelerometer",
    members=[
        MDB.FloatMember(
            name="ax",
            bits=32,
            units="m/s^2",
        ),
        MDB.FloatMember(
            name="ay",
            bits=32,
            units="m/s^2",
        ),
        MDB.FloatMember(
            name="az",
            bits=32,
            units="m/s^2",
        ),
    ],
    data_source=MDB.DataSource.LOCAL,
    short_description="Accelerometer data with x, y, z components",
)

gyroscope = MDB.AggregateParameter(
    system=rover,
    name="imu_gyroscope",
    members=[
        MDB.FloatMember(
            name="gx",
            bits=32,
            units="deg/s",
        ),
        MDB.FloatMember(
            name="gy",
            bits=32,
            units="deg/s",
        ),
        MDB.FloatMember(
            name="gz",
            bits=32,
            units="deg/s",
        ),
    ],
    data_source=MDB.DataSource.LOCAL,
    short_description="Gyroscope data with x, y, z components",
)

# Camera telemetry, one space system per robot camera with one child per image kind
# (paths from OmniLRS yamcs_naming, cameras from the OmniLRS robot yaml):
#   /Rover/camera/<camera>/<kind>/{number,name,url_storage,url_full,url_full_nginx}
#   /Rover/camera/<camera>/streaming/{<streaming telemetry fields>}   (rgba cameras)

KIND_DESCRIPTIONS = {"streaming": "streaming", "oncommand": "on-command", "depth": "depth"}


def subsystem_at(parent, path, short_description):
    """Subsystem of `parent` at the absolute MDB `path` computed by yamcs_naming."""
    parent_path, name = path.rsplit("/", 1)
    if parent_path != parent.qualified_name:
        raise ValueError(f"{path} is not a direct child of {parent.qualified_name}")
    return MDB.Subsystem(parent, name, short_description=short_description)


def image_meta_parameter(system, field, captures):
    """Image metadata parameter `field` (yamcs_naming.IMAGE_META_FIELDS), e.g. captures="cam_left streaming"."""
    if field == "number":
        return MDB.IntegerParameter(
            system=system,
            name=field,
            signed=False,
            bits=32,
            data_source=MDB.DataSource.LOCAL,
            short_description=f"Sequence identifier for {captures} captures",
        )
    descriptions = {
        "name": f"Friendly name for the {captures} capture",
        "url_storage": f"Storage endpoint for {captures} captures",
        "url_full": f"External URL for {captures} captures",
        "url_full_nginx": f"NGINX reverse proxy URL for {captures} captures",
    }
    return MDB.StringParameter(
        system=system,
        name=field,
        data_source=MDB.DataSource.LOCAL,
        short_description=descriptions[field],
    )


def streaming_parameter(system, name, camera):
    """Streaming telemetry parameter of an rgba camera (yamcs_naming.STREAMING_FIELDS)."""
    if name == "state":
        return MDB.EnumeratedParameter(
            system=system,
            name=name,
            choices=[
                (0, "OFF"),
                (1, "ON"),
            ],
            data_source=MDB.DataSource.LOCAL,
            short_description=f"{camera} streaming pipeline state",
        )
    raise ValueError(f"no MDB definition for streaming telemetry field {name!r}")


meta_systems = {}  # yamcs_naming.meta_path -> space system
for camera in cameras:
    camera_name = camera.name
    camera_system = subsystem_at(
        camera_subsystem,
        naming.camera_system(camera_name),
        short_description=f"{camera_name} camera ({', '.join(camera.type)})",
    )
    for kind in naming.camera_kinds(camera):
        captures = f"{camera_name} {KIND_DESCRIPTIONS[kind]}"
        path = naming.meta_path(camera_name, kind)
        description = f"{captures} image metadata" + (" and stream telemetry" if kind == "streaming" else "")
        meta_systems[path] = subsystem_at(camera_system, path, short_description=description)
        for field in naming.IMAGE_META_FIELDS:
            image_meta_parameter(meta_systems[path], field, captures)

    if camera.has_type("rgba"):
        for field in streaming_fields:
            path, name = naming.streaming_param(camera_name, field).rsplit("/", 1)
            streaming_parameter(meta_systems[path], name, camera_name)

# Motors
motor_current = MDB.ArrayParameter(
    system=rover,
    name="motor_current",
    data_type=MDB.FloatDataType(bits=32, units="A", minimum=0.0),
    length=4,
    data_source=MDB.DataSource.LOCAL,
    short_description="Array of motor current readings for the four wheels",
)

motor_encoder = MDB.ArrayParameter(
    system=rover,
    name="motor_encoder",
    data_type=MDB.IntegerDataType(signed=False, bits=16, minimum=0, maximum=1024, units="counts"),
    length=4,
    data_source=MDB.DataSource.LOCAL,
    short_description="Array of motor encoder positions for the four wheels",
)
# Radio
radio_rssi = MDB.IntegerParameter(
    system=rover,
    name="radio_rssi",
    signed=True,
    bits=32,
    data_source=MDB.DataSource.LOCAL,
    short_description="Radio signal strength indicator",
    units="dBm",
)

# Thermal
temperature_front = MDB.FloatParameter(
    system=rover,
    name="temperature_front",
    bits=32,
    minimum=-273.15,
    data_source=MDB.DataSource.LOCAL,
    short_description="Front temperature in Celsius",
    units="C",
)

temperature_back = MDB.FloatParameter(
    system=rover,
    name="temperature_back",
    bits=32,
    minimum=-273.15,
    data_source=MDB.DataSource.LOCAL,
    short_description="Back temperature in Celsius",
    units="C",
)

temperature_left = MDB.FloatParameter(
    system=rover,
    name="temperature_left",
    bits=32,
    minimum=-273.15,
    data_source=MDB.DataSource.LOCAL,
    short_description="Left temperature in Celsius",
    units="C",
)

temperature_right = MDB.FloatParameter(
    system=rover,
    name="temperature_right",
    bits=32,
    minimum=-273.15,
    data_source=MDB.DataSource.LOCAL,
    short_description="Right temperature in Celsius",
    units="C",
)

temperature_top = MDB.FloatParameter(
    system=rover,
    name="temperature_top",
    bits=32,
    minimum=-273.15,
    data_source=MDB.DataSource.LOCAL,
    short_description="Top temperature in Celsius",
    units="C",
)

temperature_bottom = MDB.FloatParameter(
    system=rover,
    name="temperature_bottom",
    bits=32,
    minimum=-273.15,
    data_source=MDB.DataSource.LOCAL,
    short_description="Bottom temperature in Celsius",
    units="C",
)

temperature_elec_box = MDB.FloatParameter(
    system=rover,
    name="temperature_elec_box",
    bits=32,
    minimum=-273.15,
    data_source=MDB.DataSource.LOCAL,
    short_description="Electronics box temperature in Celsius",
    units="C",
)

# Power
battery_voltage = MDB.FloatParameter(
    system=rover,
    name="battery_voltage",
    bits=32,
    minimum=0.0,
    data_source=MDB.DataSource.LOCAL,
    short_description="Battery voltage",
    units="V",
)

battery_charge = MDB.IntegerParameter(
    system=rover,
    name="battery_charge",
    signed=False,
    bits=8,
    minimum=0,
    maximum=100,
    data_source=MDB.DataSource.LOCAL,
    short_description="Battery charge percentage",
    units="%",
)

total_current_in = MDB.FloatParameter(
    system=rover,
    name="total_current_in",
    bits=32,
    minimum=0.0,
    data_source=MDB.DataSource.LOCAL,
    short_description="Total current flowing into the system",
    units="A",
)

total_current_out = MDB.FloatParameter(
    system=rover,
    name="total_current_out",
    bits=32,
    minimum=0.0,
    data_source=MDB.DataSource.LOCAL,
    short_description="Total current flowing out of the system",
    units="A",
)

current_draw_obc = MDB.FloatParameter(
    system=rover,
    name="current_draw_obc",
    bits=32,
    minimum=0.0,
    data_source=MDB.DataSource.LOCAL,
    short_description="Current consumed by On-Board Computer",
    units="A",
)

current_draw_motor_controller = MDB.FloatParameter(
    system=rover,
    name="current_draw_motor_controller",
    bits=32,
    minimum=0.0,
    data_source=MDB.DataSource.LOCAL,
    short_description="Current consumed by Motor Controller",
    units="A",
)


current_draw_camera = MDB.FloatParameter(
    system=rover,
    name="current_draw_camera",
    bits=32,
    minimum=0.0,
    data_source=MDB.DataSource.LOCAL,
    short_description="Current consumed by Camera system",
    units="A",
)

current_draw_radio = MDB.FloatParameter(
    system=rover,
    name="current_draw_radio",
    bits=32,
    minimum=0.0,
    data_source=MDB.DataSource.LOCAL,
    short_description="Current consumed by Radio system",
    units="A",
)

current_draw_eps = MDB.FloatParameter(
    system=rover,
    name="current_draw_eps",
    bits=32,
    minimum=0.0,
    data_source=MDB.DataSource.LOCAL,
    short_description="Current consumed by Electric Power System",
    units="A",
)

######################
#### Telecommands ####
######################

# System-level commands (deployments, reboot, go/nogo)

# Solar panel
deploy_arg = MDB.EnumeratedArgument(
    name="deployment",
    choices=[(0, "STOW"), (1, "DEPLOY")],
    short_description="Deployment action",
    encoding=MDB.IntegerEncoding(bits=8, scheme=MDB.IntegerEncodingScheme.UNSIGNED)
)

deploy_solar_panel = MDB.Command(
    system=system_subsystem,
    name="deploy_solar_panel",
    arguments=[deploy_arg],
    entries=[
        MDB.FixedValueEntry(
            name="command_name",
            binary=b"deploy_solar_panel",
            short_description="Command name identifier",
        ),
        MDB.ArgumentEntry(
            argument=deploy_arg,
            short_description="Deployment action argument",
        ),
    ],
    short_description="Deploy the solar panel",
)

# Reboot
reboot_mode_arg = MDB.EnumeratedArgument(
    name="reboot_mode",
    choices=[(0, "SOFT"), (1, "HARD"), (2, "SAFE_MODE")],
    short_description="Reboot mode",
    encoding=MDB.IntegerEncoding(bits=8, scheme=MDB.IntegerEncodingScheme.UNSIGNED)
)

reboot_rover = MDB.Command(
    system=system_subsystem,
    name="reboot_rover",
    arguments=[reboot_mode_arg],
    entries=[
        MDB.FixedValueEntry(
            name="command_name",
            binary=b"reboot_rover",
            short_description="Command name identifier",
        ),
        MDB.ArgumentEntry(
            argument=reboot_mode_arg,
            short_description="Reboot mode argument",
        ),
    ],
    short_description="Reboot the rover on-board computer",
)

# Deploy from lander
deploy_from_lander = MDB.Command(
    system=system_subsystem,
    name="deploy_from_lander",
    arguments=[],
    entries=[
        MDB.FixedValueEntry(
            name="command_name",
            binary=b"deploy_from_lander",
            short_description="Command name identifier",
        )
    ],
    short_description="Deploy rover from lander platform",
)

# GO/NOGO decision
go_nogo_decision_arg = MDB.EnumeratedArgument(
    name="decision",
    choices=[(0, "NOGO"), (1, "GO")],
    short_description="GO/NOGO decision",
    encoding=MDB.IntegerEncoding(bits=8, scheme=MDB.IntegerEncodingScheme.UNSIGNED)
)

go_nogo_command = MDB.Command(
    system=system_subsystem,
    name="go_nogo_command",
    arguments=[go_nogo_decision_arg],
    entries=[
        MDB.FixedValueEntry(
            name="command_name",
            binary=b"go_nogo_command",
            short_description="Command name identifier",
        ),
        MDB.ArgumentEntry(
            argument=go_nogo_decision_arg,
            short_description="GO/NOGO decision argument",
        ),
    ],
    short_description="GO/NOGO decision",
)


# Motor commands

linear_velocity_arg = MDB.FloatArgument(
    name="linear_velocity",
    bits=32,
    units="m/s",
    short_description="Linear velocity",
    encoding=MDB.FloatEncoding(bits=32)
)

distance_arg = MDB.FloatArgument(
    name="distance",
    bits=32,
    units="m",
    short_description="Distance to travel",
    encoding=MDB.FloatEncoding(bits=32)
)

drive_straight = MDB.Command(
    system=motor_subsystem,
    name="drive_straight",
    arguments=[linear_velocity_arg, distance_arg],
    entries=[
        MDB.FixedValueEntry(
            name="command_name",
            binary=b"drive_straight",
            short_description="Command name identifier",
        ),
        MDB.ArgumentEntry(
            argument=linear_velocity_arg,
            short_description="Linear velocity argument",
        ),
        MDB.ArgumentEntry(
            argument=distance_arg,
            short_description="Distance argument",
        ),
    ],
    short_description="Drive the rover straight",
)

angular_velocity_arg = MDB.FloatArgument(
    name="angular_velocity",
    bits=32,
    units="deg/s",
    short_description="Angular velocity",
    encoding=MDB.FloatEncoding(bits=32)
)

angle_arg = MDB.FloatArgument(
    name="angle",
    bits=32,
    units="deg",
    short_description="Angle to turn",
    encoding=MDB.FloatEncoding(bits=32)
)

drive_turn = MDB.Command(
    system=motor_subsystem,
    name="drive_turn",
    arguments=[angular_velocity_arg, angle_arg],
    entries=[
        MDB.FixedValueEntry(
            name="command_name",
            binary=b"drive_turn",
            short_description="Command name identifier",
        ),
        MDB.ArgumentEntry(
            argument=angular_velocity_arg,
            short_description="Angular velocity argument",
        ),
        MDB.ArgumentEntry(
            argument=angle_arg,
            short_description="Angle argument",
        ),
    ],
    short_description="Drive the rover with spot turn",
)

# Camera commands


def camera_name_argument(command_name):
    """camera_name argument: the robot cameras whose type matches the command (yamcs_naming.cameras_for_command)."""
    labels = naming.cameras_for_command(cameras, command_name)
    if not labels:
        camera_type = naming.CAMERA_COMMAND_TYPE[command_name]
        raise ValueError(f"{command_name}: no camera of type {camera_type!r} in the robot yaml")
    return MDB.EnumeratedArgument(
        name=naming.CAMERA_NAME_ARG,
        choices=list(enumerate(labels)),
        short_description=f"Camera ({naming.CAMERA_COMMAND_TYPE[command_name]} cameras of the robot)",
        encoding=MDB.IntegerEncoding(bits=8, scheme=MDB.IntegerEncodingScheme.UNSIGNED)
    )


camera_streaming_camera_arg = camera_name_argument("camera_streaming")

camera_streaming_arg = MDB.EnumeratedArgument(
    name="action",
    choices=[(0, "STOP"), (1, "START")],
    short_description="Camera streaming action",
    encoding=MDB.IntegerEncoding(bits=8, scheme=MDB.IntegerEncodingScheme.UNSIGNED)
)

camera_streaming = MDB.Command(
    system=camera_subsystem,
    name="camera_streaming",
    arguments=[camera_streaming_camera_arg, camera_streaming_arg],
    entries=[
        MDB.FixedValueEntry(
            name="command_name",
            binary=b"camera_streaming",
            short_description="Command name identifier",
        ),
        MDB.ArgumentEntry(
            argument=camera_streaming_camera_arg,
            short_description="Camera name argument",
        ),
        MDB.ArgumentEntry(
            argument=camera_streaming_arg,
            short_description="Camera action argument",
        ),
    ],
    short_description="Start or stop camera streaming",
)

camera_capture_high_camera_arg = camera_name_argument("camera_capture_high")

camera_capture_high = MDB.Command(
    system=camera_subsystem,
    name="camera_capture_high",
    arguments=[camera_capture_high_camera_arg],
    entries=[
        MDB.FixedValueEntry(
            name="command_name",
            binary=b"camera_capture_high",  # Command name as bytes
            short_description="Command name identifier",
        ),
        MDB.ArgumentEntry(
            argument=camera_capture_high_camera_arg,
            short_description="Camera name argument",
        ),
    ],
    short_description="Capture a high-resolution image",
)

camera_capture_depth_camera_arg = camera_name_argument("camera_capture_depth")

camera_capture_depth = MDB.Command(
    system=camera_subsystem,
    name="camera_capture_depth",
    arguments=[camera_capture_depth_camera_arg],
    entries=[
        MDB.FixedValueEntry(
            name="command_name",
            binary=b"camera_capture_depth",  # Command name as bytes
            short_description="Command name identifier",
        ),
        MDB.ArgumentEntry(
            argument=camera_capture_depth_camera_arg,
            short_description="Camera name argument",
        ),
    ],
    short_description="Capture a depth image",
)

# Camera settings: single config file path argument
config_path_arg = MDB.StringArgument(
    name="config_path",
    short_description="Path to the configuration file",
    encoding=MDB.StringEncoding()
)

change_camera_settings = MDB.Command(
    system=camera_subsystem,
    name="change_camera_settings",
    arguments=[config_path_arg],
    entries=[
        MDB.FixedValueEntry(
            name="command_name",
            binary=b"change_camera_settings",
            short_description="Command name identifier",
        ),
        MDB.ArgumentEntry(
            argument=config_path_arg,
            short_description="Configuration file path argument",
        ),
    ],
    short_description="Change the camera settings with a config file",
)


# Power commands

subsystem_id_arg = MDB.EnumeratedArgument(
    name="subsystem_id",
    choices=[
        (0, "CAMERA"),
        (1, "MOTOR_CONTROLLER"),
        (2, "RADIO"),
    ],
    short_description="Subsystem identifier",
    encoding=MDB.IntegerEncoding(bits=8, scheme=MDB.IntegerEncodingScheme.UNSIGNED)
)

power_state_arg = MDB.EnumeratedArgument(
    name="power_state",
    choices=[(0, "OFF"), (1, "ON")],
    short_description="Power state",
    encoding=MDB.IntegerEncoding(bits=8, scheme=MDB.IntegerEncodingScheme.UNSIGNED)
)

power_electronics = MDB.Command(
    system=power_subsystem,
    name="power_electronics",
    arguments=[subsystem_id_arg, power_state_arg],
    entries=[
        MDB.FixedValueEntry(
            name="command_name",
            binary=b"power_electronics",
            short_description="Command name identifier",
        ),
        MDB.ArgumentEntry(
            argument=subsystem_id_arg,
            short_description="Subsystem ID argument",
        ),
        MDB.ArgumentEntry(
            argument=power_state_arg,
            short_description="Power state argument",
        ),
    ],
    short_description="Power on/off electronics subsystems",
)

# ==================================
# Generate the Mission Database XML
# ==================================
# Emit an XML that conforms to XTCE
# print(rover.dumps())

write_mdb(rover, "rover")

# ==================================
# Image buckets (etc/yamcs.yaml)
# ==================================
# Every bucket OmniLRS uploads images to: the cameras' buckets and the role and payload buckets of the controller yaml
write_image_buckets(
    [name for name, _ in naming.image_buckets(cameras, yamcs_tmtc)],
    prefix=naming.BUCKET_PREFIX,
)
