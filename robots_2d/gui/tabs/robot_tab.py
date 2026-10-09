import numpy as np
from dash import dcc, html


class RobotTab(dcc.Tab):

    def __init__(self):

        children = [
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
        ]

        super().__init__(
            label = "Robot",
            children = children
        )