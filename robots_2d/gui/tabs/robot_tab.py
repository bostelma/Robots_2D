import numpy as np
from dash import dcc, html

from robots_2d.classes.robot import Robot
from robots_2d.robots.robot_2r import Robot2R
from robots_2d.robots.robot_3r import Robot3R


class RobotTab(dcc.Tab):

    def __init__(self):

        children = [

            html.Br(),

            # Robot Selection
            html.Div(
                [
                    html.Label(
                        "Robot Selection:",
                        style = {
                            "marginRight": "10px"
                        }
                    ),
                    html.Div(
                        dcc.Dropdown(
                            id = "robot-type",
                            options = [
                                {"label": "Robot2R", "value": "Robot2R"},
                                {"label": "Robot3R", "value": "Robot3R"},
                            ],
                            value = "Robot2R",
                            clearable = False,
                        ),
                        style = {
                            "flex": "1",
                            "minWidth": "0",
                        }
                    ),
                ],
                style = {
                    "display": "flex",
                    "alignItems": "center",
                    "gap": "10px",
                    "border": "1px solid #b0b0b0",
                    "padding": "15px",
                    "borderRadius": "5px",
                    "width": "100%",
                    "boxSizing": "border-box"
                }
            ),

            html.Br(),

            # Joint angles
            html.Div(
                [
                    html.Div(
                        [
                            html.Label(
                                "Link length 1",
                                style = {
                                    "whiteSpace": "nowrap",
                                    "flexShrink": 0,
                                    "marginRight": "10px"
                                }
                            ),
                            dcc.Input(
                                id = "link-length-1",
                                type = "number",
                                value = 1.0,
                            ),
                        ],
                        style = {
                            "display": "flex",
                        }
                    ),

                    html.Br(),

                    html.Div(
                        [
                            html.Label(
                                "Link length 2",
                                style = {
                                    "whiteSpace": "nowrap",
                                    "flexShrink": 0,
                                    "marginRight": "10px"
                                }
                            ),
                            dcc.Input(
                                id = "link-length-2",
                                type = "number",
                                value = 1.0,
                            ),
                        ],
                        style = {
                            "display": "flex",
                        }
                    ),

                    html.Br(),

                    html.Div(
                        [
                            html.Label(
                                "Link length 3",
                                style = {
                                    "whiteSpace": "nowrap",
                                    "flexShrink": 0,
                                    "marginRight": "10px"
                                }
                            ),
                            dcc.Input(
                                id = "link-length-3",
                                type = "number",
                                value = 1.0,
                            ),
                        ],
                        id = "link-length-3-container",
                        style = {
                            "display": "flex",
                        }
                    ),
                ],
                style = {
                    "gap": "10px",
                    "border": "1px solid #b0b0b0",
                    "padding": "15px",
                    "borderRadius": "5px",
                    "width": "100%",
                    "boxSizing": "border-box"
                }
            ),

            html.Br(),

            # Joint angles
            html.Div(
                [
                    html.Div(
                        [
                            html.Label(
                                "Joint 1",
                                style = {
                                    "whiteSpace": "nowrap",
                                    "flexShrink": 0,
                                    "marginRight": "10px"
                                }
                            ),
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
                        ],
                        style = {
                            "display": "flex",
                        }
                    ),

                    html.Br(),

                    html.Div(
                        [
                            html.Label(
                                "Joint 2",
                                style = {
                                    "whiteSpace": "nowrap",
                                    "flexShrink": 0,
                                    "marginRight": "10px"
                                }
                            ),
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
                        ],
                        style = {
                            "display": "flex",
                        }
                    ),

                    html.Br(),

                    html.Div(
                        [
                            html.Label(
                                "Joint 3",
                                style = {
                                    "whiteSpace": "nowrap",
                                    "flexShrink": 0,
                                    "marginRight": "10px"
                                }
                            ),
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
                        ],
                        id = "joint-3-container",
                        style = {
                            "display": "flex",
                        }
                    ),
                ],
                style = {
                    "gap": "10px",
                    "border": "1px solid #b0b0b0",
                    "padding": "15px",
                    "borderRadius": "5px",
                    "width": "100%",
                    "boxSizing": "border-box"
                }
            ),
        ]

        super().__init__(
            label = "Robot",
            children = children
        )

    @classmethod
    def create_robot(
        cls,
        robot_type: str,
        l1 = 1.0,
        l2 = 1.0,
        l3 = 1.0
    ) -> Robot:

        if robot_type == "Robot2R":
            return Robot2R(
                l1 = l1,
                l2 = l2
            )

        if robot_type == "Robot3R":
            return Robot3R(
            l1 = l1,
            l2 = l2,
            l3 = l3
        )

        raise ValueError(
            f"Unknown robot type: {robot_type}"
        )