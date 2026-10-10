from dash import dcc, html


class ManipulabilityTab(dcc.Tab):

    def __init__(self):

        children = [

            html.Br(),

            # Manipulability ellipse toggle
            html.Div(
                [
                    dcc.Checklist(
                        id="show-manipulability",
                        options=[
                            {
                                "label": "Show Manipulability Ellipse",
                                "value": "show",
                            }
                        ],
                        value = [],
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

            # Manipulability measures
            html.Div(
                [
                    html.Div(
                        [
                            html.Label(
                                "Manipulability Measure 1",
                                style = {
                                    "whiteSpace": "nowrap",
                                    "flexShrink": 0,
                                    "marginRight": "10px"
                                }
                            ),
                            dcc.Input(
                                id = "manipulability-measure-1",
                                type = "number",
                                value = 0.0,
                                readOnly = True,
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
                                "Manipulability Measure 2",
                                style = {
                                    "whiteSpace": "nowrap",
                                    "flexShrink": 0,
                                    "marginRight": "10px"
                                }
                            ),
                            dcc.Input(
                                id = "manipulability-measure-2",
                                type = "number",
                                value = 0.0,
                                readOnly = True,
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
                                "Manipulability Measure 3",
                                style = {
                                    "whiteSpace": "nowrap",
                                    "flexShrink": 0,
                                    "marginRight": "10px"
                                }
                            ),
                            dcc.Input(
                                id = "manipulability-measure-3",
                                type = "number",
                                value = 0.0,
                                readOnly = True,
                            ),
                        ],
                        style = {
                            "display": "flex",
                            "alignItems": "center",
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
            label = "Manipulability",
            children = children
        )