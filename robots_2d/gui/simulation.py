import numpy as np
import plotly.graph_objects as go
from dash import Input, Output, State, ctx, dcc, html

from robots_2d.classes.joint import JointType
from robots_2d.classes.robot import Robot
from robots_2d.gui.tabs.robot_tab import RobotTab


def create_robot_figure(
        robot: Robot,
        show_manipulability: bool = False
    ) -> go.Figure:

    poses = robot.forward_kinematics()

    fig = go.Figure()

    # Links
    for link in robot.links:

        if link.name == "link_end":
            continue

        T = poses[link.name]

        p_start = np.asarray(
            T * np.array([0.0, 0.0, 0.0])
        ).ravel()

        p_end = np.asarray(
            T * np.array([link.length, 0.0, 0.0])
        ).ravel()

        fig.add_trace(
            go.Scatter(
                x = [float(p_start[0]), float(p_end[0])],
                y = [float(p_start[1]), float(p_end[1])],
                mode = "lines",
                line = {
                    "color": "black",
                    "width": 8,
                },
                showlegend = False,
                hoverinfo = "skip",
            )
        )

    # Revolute joints
    joint_x = []
    joint_y = []

    for joint in robot.joints:

        if joint.type != JointType.REVOLUTE:
            continue

        T = poses[joint.child.name]

        p = np.asarray(
            T * np.array([0.0, 0.0, 0.0])
        ).ravel()

        joint_x.append(float(p[0]))
        joint_y.append(float(p[1]))

    fig.add_trace(
        go.Scatter(
            x = joint_x,
            y = joint_y,
            mode = "markers",
            marker = {
                "color": "red",
                "size": 16,
            },
            showlegend = False,
            hoverinfo = "skip",
        )
    )

    # End effector
    T = poses["link_end"]
    p = np.asarray(
        T * np.array([0.0, 0.0, 0.0])
    ).ravel()

    fig.add_trace(
        go.Scatter(
            x = [float(p[0])],
            y = [float(p[1])],
            mode = "markers",
            marker = {
                "color": "green",
                "size": 16,
            },
            showlegend = False,
            hoverinfo = "skip",
        )
    )

    if show_manipulability:

        center, width, height, angle = robot.manipulability_ellipse()

        theta = np.linspace(0.0, 2.0 * np.pi, 100)

        # Full axis lengths -> semi-axis lengths
        a = float(width) / 2.0
        b = float(height) / 2.0

        # Angle is returned in degrees and cloclwise
        angle = -np.deg2rad(angle)

        # Ellipse aligned with x/y axes
        x = a * np.cos(theta)
        y = b * np.sin(theta)

        # Rotate ellipse counterclockwise from +x
        x_rot = x * np.cos(angle) - y * np.sin(angle)
        y_rot = x * np.sin(angle) + y * np.cos(angle)

        # Translate to end-effector position
        x_rot += float(center[0])
        y_rot += float(center[1])

        fig.add_trace(
            go.Scatter(
                x=x_rot,
                y=y_rot,
                mode="lines",
                line={
                    "color": "blue",
                    "width": 2,
                },
                showlegend=False,
                hoverinfo="skip",
            )
        )

    # Workspace / axes
    total_length = sum(link.length for link in robot.links)
    limit = 1.5 * total_length

    fig.update_layout(
        paper_bgcolor="white",
        plot_bgcolor="white",

        xaxis = {
            "range": [-limit, limit],
            "scaleanchor": "y",
            "scaleratio": 1,

            # Axis
            "showline": True,
            "linecolor": "grey",
            "linewidth": 1,
            "mirror": True,

            # Zero axis
            "zeroline": True,
            "zerolinecolor": "grey",
            "zerolinewidth": 1,

            # Grid
            "showgrid": True,
            "gridcolor": "lightgrey",
            "gridwidth": 1,
        },

        yaxis = {
            "range": [-limit, limit],

            # Axis
            "showline": True,
            "linecolor": "grey",
            "linewidth": 1,
            "mirror": True,

            # Zero axis
            "zeroline": True,
            "zerolinecolor": "grey",
            "zerolinewidth": 1,

            # Grid
            "showgrid": True,
            "gridcolor": "lightgrey",
            "gridwidth": 1,
        },

        margin={"l": 40, "r": 40, "t": 40, "b": 40},
        showlegend=False,
    )

    return fig

class Simulation(html.Div):

    def __init__(self, app):

        self.app = app
        self.current_frame = 0

        self.register_callbacks()

        children = [
            dcc.Graph(
                id = "robot-graph",
            ),

            html.Br(),

            html.Div(
                [
                    html.Button(
                        id = "play",
                        children = "Play",
                    ),

                    html.Button(
                        id = "pause",
                        children = "Pause",
                    ),

                    html.Br(),

                    dcc.Slider(
                        id = "frame",
                        min = 0,
                        max = 0,
                        step = 1,
                        value = 0,
                    ),
                ],
                style = {
                    "display": "flex",
                    "gap": "10px",
                }
            ),

            dcc.Interval(
                id = "timer",
                interval = 100,
                n_intervals = 0,
                max_intervals = 1000000,
                disabled = True
            )
        ]

        style = {
            "width": "50%",
            "height": "100vh",
            "padding": "10px",
            "boxSizing": "border-box",
        }

        super().__init__(
            children = children,
            style = style
        )


    def register_callbacks(self):

        # Update the frame slider and timer
        @self.app.callback(
                Output("frame", "value"),
                Output("timer", "disabled"),
                Output("timer", "n_intervals"),
                State("frame", "value"),
                State("timer", "disabled"),
                Input("timer", "n_intervals"),
                Input("robot-data", "data"),
                Input("play", "n_clicks"),
                Input("pause", "n_clicks"),
                prevent_initial_call = True,
        )
        def process_controls(frame, disabled, timer_interval, robot_data, *_):

            pressed_play = ctx.triggered_id == "play"
            pressed_pause = ctx.triggered_id == "pause"
            timer_event = ctx.triggered_id == "timer"
            new_robot = ctx.triggered_id == "robot-data"

            # Handle pressing of the pause button
            if pressed_pause:

                # Pause simulation
                disabled = True

            # Handle pressing of the play button
            if pressed_play and disabled:

                # Reset if necessary
                if frame == len(robot_data["qs"]) - 1:
                    frame = 0
                    timer_interval = 0

                disabled = False
                    
            # Handle timer event
            if timer_event:

                # Increase simulation timer
                frame += 1

                # Stop at the end of the simulation
                if frame >= len(robot_data["qs"]) - 1:
                    frame = min(frame, len(robot_data["qs"]) - 1)
                    disabled = True

            # Handle new robot data
            if new_robot:

                # Reset everything
                disabled = True
                frame = 0
                timer_interval = 0

            return frame, disabled, timer_interval

        # Process new robot data
        @self.app.callback(
                Output("frame", "max"),
                Input("robot-data", "data"),
                prevent_initial_call = True,
        )
        def process_robot_data(data):

            return len(data["qs"]) - 1

        # Process a new frame
        @self.app.callback(
                Output("robot-graph", "figure"),
                Output("manipulability-measure-1", "value"),
                Output("manipulability-measure-2", "value"),
                Output("manipulability-measure-3", "value"),
                Input("frame", "value"),
                Input("show-manipulability", "value"),
                State("robot-data", "data"),
                prevent_initial_call = True,
        )
        def update_figure(frame, show_manipulability, robot_data):

            robot = RobotTab.create_robot(
                robot_data["robot_type"],
                **robot_data["link_lengths"],
            )
            robot.move_joints(robot_data["qs"][frame])

            figure = create_robot_figure(
                robot,
                show_manipulability = show_manipulability
            )

            return (
                figure,
                robot.manipulability_measure_1(),
                robot.manipulability_measure_2(),
                robot.manipulability_measure_3(),
            )