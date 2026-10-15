import numpy as np
import plotly.graph_objects as go
from dash import Dash, Input, Output, State, dcc, html

from robots_2d.classes.joint import JointType
from robots_2d.classes.robot import Robot
from robots_2d.gui.simulation import Simulation
from robots_2d.gui.tabs.ik_tab import IKTab
from robots_2d.gui.tabs.manipulability_tab import ManipulabilityTab
from robots_2d.gui.tabs.robot_tab import RobotTab

# ---------------------------------------------------------------------------
# Plotly figure
# ---------------------------------------------------------------------------




# ---------------------------------------------------------------------------
# Dash application
# ---------------------------------------------------------------------------

app = Dash(__name__)

app.layout = html.Div(
    [

        # Data Storage
        dcc.Store(
            id = "robot-data",
            data = {}
        ),

        # Left Side: Main figure
        Simulation(app),

        # Right Side: Tabs and graphs
        html.Div(
            [
                # Top-right tabs
                dcc.Tabs(
                    id="tabs",
                    children=[
                        RobotTab(),
                        ManipulabilityTab(),
                        IKTab(),
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
    Output("link-length-3-container", "style"),
    Output("joint-3-container", "style"),
    Output("ik-angle-container", "style"),
    Output("robot-data", "data"),
    Input("robot-type", "value"),
    Input("link-length-1", "value"),
    Input("link-length-2", "value"),
    Input("link-length-3", "value"),
    Input("joint-1", "value"),
    Input("joint-2", "value"),
    Input("joint-3", "value"),
)
def update_robot(
    robot_type,
    l1,
    l2,
    l3,
    q1,
    q2,
    q3,
):

    if robot_type == "Robot2R":

        q = np.array([q1, q2], dtype=float)
        link_length_3_style = {
            "display": "none"
        }
        joint_3_style = {
            "display": "none"
        }
        ik_angle_style = {
            "display": "none"
        }

    elif robot_type == "Robot3R":

        q = np.array([q1, q2, q3], dtype=float)
        link_length_3_style = {
            "display": "flex",
            "alignItems": "center",
        }
        joint_3_style = {
            "display": "flex"
        }
        ik_angle_style = {
            "display": "flex",
            "alignItems": "center",
        }

    else:
        raise ValueError(
            f"Unknown robot type: {robot_type}"
        )

    return (
        link_length_3_style,
        joint_3_style,
        ik_angle_style,
        {
            "robot_type": robot_type,
            "link_lengths": {
                "l1": float(l1),
                "l2": float(l2),
                "l3": float(l3),
            },
            "qs": [q.tolist()],
        }
    )

@app.callback(
    Output("joint-1", "value"),
    Output("joint-2", "value"),
    Output("joint-3", "value"),
    Output("ik-message", "children"),
    State("robot-type", "value"),
    State("link-length-1", "value"),
    State("link-length-2", "value"),
    State("link-length-3", "value"),
    State("ik-x", "value"),
    State("ik-y", "value"),
    State("ik-angle", "value"),
    State("joint-1", "value"),
    State("joint-2", "value"),
    State("joint-3", "value"),
    Input("ik-compute", "n_clicks"),
    prevent_initial_call=True,
)
def compute_ik(robot_type, l1, l2, l3, x, y, theta, q1, q2, q3, _):

    ik_robot = RobotTab.create_robot(
        robot_type,
        l1=l1,
        l2=l2,
        l3=l3,
    )

    if robot_type == "Robot2R":
        initial_q = np.array([q1, q2], dtype=float)

    elif robot_type == "Robot3R":
        initial_q = np.array([q1, q2, q3], dtype=float)

    else:
        raise ValueError(f"Unknown robot type: {robot_type}")

    ik_robot.move_joints(initial_q)

    try:
        if robot_type == "Robot2R":
            qs = ik_robot.inverse_kinematics_num(np.array([x, y]))
            return qs[0], qs[1], 0.0, "Found valid solution!"

        qs = ik_robot.inverse_kinematics_num(
            np.array([x, y]),
            theta=theta,
        )
        return qs[0], qs[1], qs[2], "Found valid solution!"
    except RuntimeError:
        return q1, q2, q3, "Failed to compute IK!"

if __name__ == "__main__":
    app.run(debug=False)