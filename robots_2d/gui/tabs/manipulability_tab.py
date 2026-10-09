from dash import dcc, html


class ManipulabilityTab(dcc.Tab):

    def __init__(self):

        children = [
            html.Div(
                [
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
            ),
        ]

        super().__init__(
            label = "Manipulability",
            children = children
        )