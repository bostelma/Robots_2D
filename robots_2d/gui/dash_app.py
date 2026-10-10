import numpy as np
import plotly.graph_objects as go
from dash import Dash, Input, Output, dcc, html

from robots_2d.classes.joint import JointType
from robots_2d.classes.robot import Robot
from robots_2d.gui.tabs.manipulability_tab import ManipulabilityTab
from robots_2d.gui.tabs.robot_tab import RobotTab

# ---------------------------------------------------------------------------
# Plotly figure
# ---------------------------------------------------------------------------

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


# ---------------------------------------------------------------------------
# Dash application
# ---------------------------------------------------------------------------

app = Dash(__name__)

app.layout = html.Div(
    [

        # Left Side: Main figure
        html.Div(
            [
                dcc.Graph(
                    id = "robot-graph",
                )
            ],
            style={
                "width": "50%",
                "height": "100vh",
                "padding": "10px",
                "boxSizing": "border-box",
            },
        ),

        # Right Side: Tabs and graphs
        html.Div(
            [
                # Top-right tabs
                dcc.Tabs(
                    id="tabs",
                    children=[
                        RobotTab(),
                        ManipulabilityTab(),
                    ],
                ),
            ],
            style={
                "width": "50%",
                "height": "100vh",
                "padding": "10px",
                "boxSizing": "border-box",
            },
        ),
    ],
    style={
        "display": "flex",
        "flexDirection": "row",
        "width": "100%",
        "height": "100vh",
        "overflow": "hidden",
    },
)


# ---------------------------------------------------------------------------
# Callback
# ---------------------------------------------------------------------------

@app.callback(
    Output("robot-graph", "figure"),
    Output("link-length-3-container", "style"),
    Output("joint-3-container", "style"),
    Output("manipulability-measure-1", "value"),
    Output("manipulability-measure-2", "value"),
    Output("manipulability-measure-3", "value"),
    Input("robot-type", "value"),
    Input("link-length-1", "value"),
    Input("link-length-2", "value"),
    Input("link-length-3", "value"),
    Input("joint-1", "value"),
    Input("joint-2", "value"),
    Input("joint-3", "value"),
    Input("show-manipulability", "value"),
)
def update_robot(
    robot_type,
    l1,
    l2,
    l3,
    q1,
    q2,
    q3,
    show_manipulability,
):

    robot = RobotTab.create_robot(
        robot_type,
        l1 = l1,
        l2 = l2,
        l3 = l3
    )

    if robot_type == "Robot2R":

        q = np.array([q1, q2])
        link_length_3_style = {
            "display": "none"
        }
        joint_3_style = {
            "display": "none"
        }

    elif robot_type == "Robot3R":

        q = np.array([q1, q2, q3])
        link_length_3_style = {
            "display": "flex"
        }
        joint_3_style = {
            "display": "flex"
        }

    else:
        raise ValueError(
            f"Unknown robot type: {robot_type}"
        )

    robot.move_joints(q)

    figure = create_robot_figure(
        robot,
        show_manipulability = show_manipulability
    )

    return (
        figure,
        link_length_3_style,
        joint_3_style,
        robot.manipulability_measure_1(),
        robot.manipulability_measure_2(),
        robot.manipulability_measure_3(),
    )


if __name__ == "__main__":
    app.run(debug=False)