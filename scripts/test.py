# imports
import plotly.graph_objs as go
import plotly.express as px
import pandas as pd
import numpy as np

# sample data in a pandas dataframe
np.random.seed(1)
df=pd.DataFrame(dict(A=np.random.uniform(low=-1, high=2, size=25).tolist(),
                    B=np.random.uniform(low=-4, high=3, size=25).tolist(),
                    C=np.random.uniform(low=-1, high=3, size=25).tolist(),
                    ))
df = df.cumsum()

# define colors as a list 
colors = px.colors.qualitative.Plotly

# convert plotly hex colors to rgba to enable transparency adjustments
def hex_rgba(hex, transparency):
    col_hex = hex.lstrip('#')
    col_rgb = list(int(col_hex[i:i+2], 16) for i in (0, 2, 4))
    col_rgb.extend([transparency])
    areacol = tuple(col_rgb)
    return areacol

rgba = [hex_rgba(c, transparency=0.2) for c in colors]
colCycle = ['rgba'+str(elem) for elem in rgba]

# Make sure the colors run in cycles if there are more lines than colors
def next_col(cols):
    while True:
        for col in cols:
            yield col
line_color=next_col(cols=colCycle)

# plotly  figure
fig = go.Figure()

# add line and shaded area for each series and standards deviation
print(df)
for i, col in enumerate(df):
    new_col = next(line_color)
    x = list(df.index.values+1)
    print(x[::-1])
    y1 = df[col]
    y1_upper = [(y + np.std(df[col])) for y in df[col]]
    y1_lower = [(y - np.std(df[col])) for y in df[col]]
    y1_lower = y1_lower[::-1]

    # standard deviation area
    fig.add_traces(go.Scatter(x=x+x[::-1],
                                y=y1_upper+y1_lower,
                                fill='tozerox',
                                fillcolor=new_col,
                                line=dict(color='rgba(255,255,255,0)'),
                                showlegend=False,
                                name=col))

    # line trace
    fig.add_traces(go.Scatter(x=x,
                              y=y1,
                              line=dict(color=new_col, width=2.5),
                              mode='lines',
                              name=col)
                                )
# set x-axis
fig.update_layout(xaxis=dict(range=[1,len(df)]))

fig.show()