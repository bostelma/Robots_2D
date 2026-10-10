import numpy as np
from dash import dcc, html


class IKTab(dcc.Tab):

    def __init__(self):

        children = [

            html.Br(),

            html.Div(
                [
                    html.Div(
                        [
                            html.Label(
                                "X",
                                style = {
                                    "whiteSpace": "nowrap",
                                    "flexShrink": 0,
                                    "marginRight": "10px"
                                }
                            ),
                            dcc.Input(
                                id = "ik-x",
                                type = "number",
                                value = 1.0,
                            ),
                        ],
                        style = {
                            "display": "flex",
                            "alignItems": "center",
                        }
                    ),

                    html.Br(),

                    html.Div(
                        [
                            html.Label(
                                "Y",
                                style = {
                                    "whiteSpace": "nowrap",
                                    "flexShrink": 0,
                                    "marginRight": "10px"
                                }
                            ),
                            dcc.Input(
                                id = "ik-y",
                                type = "number",
                                value = 1.0,
                            ),
                        ],
                        style = {
                            "display": "flex",
                            "alignItems": "center",
                        }
                    ),

                    html.Br(),

                    html.Div(
                        [
                            html.Label(
                                "Angle",
                                style = {
                                    "whiteSpace": "nowrap",
                                    "flexShrink": 0,
                                    "marginRight": "10px"
                                }
                            ),
                            dcc.Slider(
                                id = "ik-angle",
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
                        id = "ik-angle-container",
                        style = {
                            "display": "None",
                            "alignItems": "center",
                        }
                    ),

                    html.Br(),

                    html.Button(
                        id = "ik-compute",
                        children = "Compute",
                        style = {
                            "width": "100%"
                        }
                    ),

                    html.Br(),
                    html.Br(),

                    html.Label(
                        "Press Button to start ...",
                        id = "ik-message"
                    )

                ], style = {
                    "gap": "10px",
                    "border": "1px solid #b0b0b0",
                    "padding": "15px",
                    "borderRadius": "5px",
                    "width": "100%",
                    "boxSizing": "border-box"
                }
            )
        ]

        super().__init__(
            label = "IK",
            children = children
        )