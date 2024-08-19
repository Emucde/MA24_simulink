import scipy.io as sio
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import plotly.offline as py
from bs4 import BeautifulSoup
import os
import webbrowser


def plot_solution(subplot_data, save_plot=False, file_name='plot_saved', plot_fig=True):
    subplot_number = len(subplot_data)
    sig_labels = np.empty(24, dtype=object)
    for i in range(0, subplot_number):
        sig_label     = subplot_data[i][0][0][0][0]
        if len(subplot_data[i][0]) > 1:
            sig_labels[i] = sig_label[:-2]
        else:
            sig_labels[i] = sig_label
        #row=1+np.mod(i,4), col=1+int(np.floor(i/4)
    sig_labels = sig_labels.reshape(6,4).T.flatten().tolist()
    
    # Create a Plotly subplot
    fig = make_subplots(rows=4, cols=int(subplot_number/4), shared_xaxes=False, vertical_spacing=0.05, horizontal_spacing=0.035, subplot_titles=sig_labels)
    
    # Plot tdata
    for i in range(0, subplot_number):
        signal_number = len(subplot_data[i][0])
        for j in range(0, signal_number):
            sig_label     = subplot_data[i][0][j][0][0]
            sig_xdata     = subplot_data[i][0][j][1][0]
            sig_ydata     = subplot_data[i][0][j][2][0]
            sig_linestyle = subplot_data[i][0][j][3][0]
            sig_color     = 255*subplot_data[i][0][j][4][0]

            if sig_linestyle == '-':
                line_style = dict(width=1, color=f"rgb({','.join(map(str, sig_color))})", dash='solid')
            elif sig_linestyle == '--':
                line_style = dict(width=1, color=f"rgb({','.join(map(str, sig_color))})", dash='dash')

            act_row = 1+np.mod(i,4)
            act_col = 1+int(np.floor(i/4))
            fig.add_trace(go.Scatter(x=sig_xdata, y=sig_ydata, name=sig_label, line = line_style, hoverinfo = 'x+y+text', hovertext=sig_label), row=act_row, col=act_col)
            if act_row == 4:
                fig.update_xaxes(title_text='t (s)', row=act_row, col=act_col)

    # fig.update_layout(plot_bgcolor='#1e1e1e', paper_bgcolor='#1e1e1e', font=dict(color='#ffffff'), legend=dict(orientation='h'))
    fig.update_layout(
        plot_bgcolor='#101010',  # Set plot background color
        paper_bgcolor='#1e1e1e',  # Set paper background color
        font=dict(color='#ffffff'),  # Set font color
        legend=dict(orientation='h', yanchor='middle', y=10, yref='container'),  # Set legend orientation
        hovermode = 'closest',
        margin=dict(l=10, r=10, t=50, b=70),
        height=1080,
        # legend_indentation = 0,
        # margin_pad=0,
        # Gridline customization for all subplots
        **{f'xaxis{i}': dict(gridwidth=1, gridcolor='#757575', linecolor='#757575', zerolinecolor='#757575', zerolinewidth=1) for i in range(1, subplot_number+1)},
        **{f'yaxis{i}': dict(gridwidth=1, gridcolor='#757575', linecolor='#757575', zerolinecolor='#757575', zerolinewidth=1) for i in range(1, subplot_number+1)}
    )

    fig.update_layout(
        **{f'xaxis{i}': dict(showticklabels=False) for i in range(1, subplot_number+1-6)}
    )

    fig.update_xaxes(matches='x', autorange=True)

    if(plot_fig):
        fig.show()

    if(save_plot):
        py.plot(fig, filename=file_name, include_mathjax='cdn', auto_open=False)
        with open(file_name, 'r', encoding='utf-8') as file:
            html_content = file.read()
            soup = BeautifulSoup(html_content, 'html.parser')
            first_script_tag = soup.find('script')
            if first_script_tag:
                new_script = soup.new_tag('script')
                new_script.string = "document.body.style.background='#1e1e1e'"
                first_script_tag.insert_after(new_script)
                with open(file_name, 'w', encoding='utf-8') as file:
                    file.write(str(soup))
        webbrowser.open('file://' + file_name)

###################### MAIN ######################
# Laden der .mat-Datei
folderpath = "/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/mails/meeting_22aug/example2_jointspace"
mat_file_name = "240815_messung5_mpcv1_dyn.mat"
mat_file_path = os.path.join(folderpath, mat_file_name)
outputname = mat_file_name[:-4] + '.html'
output_file_path = os.path.join(folderpath, outputname)

data = sio.loadmat(mat_file_path)

subplot_data = data['signals']

# plot_solution(subplot_data)
plot_solution(subplot_data, plot_fig = False, save_plot=True, file_name=output_file_path)