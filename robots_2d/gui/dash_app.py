import numpy as np
import plotly.graph_objects as go
from dash import Dash, Input, Output, dcc, html

from robots_2d.classes.joint import JointType
from robots_2d.classes.robot import Robot
from robots_2d.robots.robot_2r import Robot2R
from robots_2d.robots.robot_3r import Robot3R

# ---------------------------------------------------------------------------
# Robot construction
# ---------------------------------------------------------------------------

def create_robot(robot_type: str) -> Robot:

    if robot_type == "Robot2R":
        return Robot2R(
            l1 = 1.0,
            l2 = 1.0
        )

    if robot_type == "Robot3R":
        return Robot3R(
        l1 = 1.0,
        l2 = 1.0,
        l3 = 1.0
    )

    raise ValueError(
        f"Unknown robot type: {robot_type}"
    )


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

        p_start = T * np.array([0.0, 0.0, 0.0])
        p_end = T * np.array([link.length, 0.0, 0.0])

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

        p = T * np.array([0.0, 0.0, 0.0])

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
    p = T * np.array([0.0, 0.0, 0.0])

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

        # Angle is returned in degrees
        angle = np.deg2rad(angle)

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
        html.H2("2D Robot"),

        html.Label("Robot"),
        dcc.Dropdown(
            id = "robot-type",
            options = [
                {"label": "Robot2R", "value": "Robot2R"},
                {"label": "Robot3R", "value": "Robot3R"},
            ],
            value = "Robot2R",
            clearable = False,
        ),

        html.Br(),

        html.Label("Joint 1"),
        dcc.Slider(
            id = "joint-1",
            min = -np.pi,
            max = np.pi,
            step = 0.01,
            value = 0.0,
            marks = {
                -np.pi: "-π",
                -np.pi / 2: "-π/2",
                0: "0",
                np.pi / 2: "π/2",
                np.pi: "π",
            },
        ),

        html.Br(),

        html.Label("Joint 2"),
        dcc.Slider(
            id = "joint-2",
            min = -np.pi,
            max = np.pi,
            step = 0.01,
            value = 0.0,
            marks = {
                -np.pi: "-π",
                -np.pi / 2: "-π/2",
                0: "0",
                np.pi / 2: "π/2",
                np.pi: "π",
            },
        ),

        html.Br(),

        html.Div(
            id = "joint-3-container",
            children = [
                html.Label("Joint 3"),
                dcc.Slider(
                    id = "joint-3",
                    min = -np.pi,
                    max = np.pi,
                    step = 0.01,
                    value = 0.0,
                    marks = {
                        -np.pi: "-π",
                        -np.pi / 2: "-π/2",
                        0: "0",
                        np.pi / 2: "π/2",
                        np.pi: "π",
                    },
                ),
                html.Br(),
            ],
            style = {
                "display": "none"
            },
        ),

        html.Div(
            [
                html.H4("Manipulability"),

                dcc.Checklist(
                    id="show-manipulability",
                    options=[
                        {
                            "label": "Show manipulability ellipse",
                            "value": "show",
                        }
                    ],
                    value=[],
                ),

                html.Br(),

                html.Label("Manipulability Measure 1"),
                dcc.Input(
                    id="manipulability-measure-1",
                    type="number",
                    value=0.0,
                    readOnly=True,
                ),

                html.Br(),
                html.Br(),

                html.Label("Manipulability Measure 2"),
                dcc.Input(
                    id="manipulability-measure-2",
                    type="number",
                    value=0.0,
                    readOnly=True,
                ),

                html.Br(),
                html.Br(),

                html.Label("Manipulability Measure 3"),
                dcc.Input(
                    id="manipulability-measure-3",
                    type="number",
                    value=0.0,
                    readOnly=True,
                ),
            ],
            style={
                "border": "1px solid lightgrey",
                "padding": "15px",
                "marginTop": "20px",
                "marginBottom": "20px",
                "width": "300px",
            },
        ),

        dcc.Graph(
            id = "robot-graph",
        ),
    ],
)


# ---------------------------------------------------------------------------
# Callback
# ---------------------------------------------------------------------------

@app.callback(
    Output("robot-graph", "figure"),
    Output("joint-3-container", "style"),
    Output("manipulability-measure-1", "value"),
    Output("manipulability-measure-2", "value"),
    Output("manipulability-measure-3", "value"),
    Input("robot-type", "value"),
    Input("joint-1", "value"),
    Input("joint-2", "value"),
    Input("joint-3", "value"),
    Input("show-manipulability", "value"),
)
def update_robot(
    robot_type,
    q1,
    q2,
    q3,
    show_manipulability,
):

    robot = create_robot(robot_type)

    if robot_type == "Robot2R":

        q = np.array([q1, q2])
        joint_3_style = {
            "display": "none"
        }

    elif robot_type == "Robot3R":

        q = np.array([q1, q2, q3])
        joint_3_style = {
            "display": "block"
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
        joint_3_style,
        robot.manipulability_measure_1(),
        robot.manipulability_measure_2(),
        robot.manipulability_measure_3(),
    )


if __name__ == "__main__":
    app.run(debug=False)