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
        autoscale_code='''
        rec_time = 100; //ms
        function autoscale_function(){
            //console.log('run');
            graphDiv = document.querySelector('.plotly-graph-div');
            //console.log(graphDiv);
            if(graphDiv == null || graphDiv.on == undefined)
            {
                setTimeout(autoscale_function, rec_time);
            }
            else
            {
                var on_event=true;

                function test(eventdata){
                    //console.log(eventdata);
                    
                    graphDiv = document.querySelector('.plotly-graph-div');
                    
                    if(on_event==true)
                    {
                        on_event=false;
                        labels = graphDiv.layout.annotations;
                        update={};

                        labels.forEach(function(act_label, i){

                            trace = graphDiv.data.filter(trace => trace.name.includes(labels[i].text));

                            xrange = graphDiv.layout.xaxis.range;
                            yaxisName = i === 0 ? 'yaxis' : `yaxis${i + 1}`;
                            yrange = graphDiv.layout[yaxisName].range;
                            filteredIndices = trace[0].x.map((x, index) => x >= xrange[0] && x <= xrange[1] ? index : -1).filter(index => index !== -1);
                            //filteredX = filteredIndices.map(index => trace[0].x[index]);

                            yaxis_change = Object.keys(eventdata).some(key => key.includes('yaxis'));
                            xaxis_change = Object.keys(eventdata).some(key => key.includes('xaxis'));

                            g_ymax=-Infinity;
                            g_ymin=Infinity;
                            trace.forEach(function(el,id){
                                filteredY = filteredIndices.map(index => el.y[index]);
                                if( (yaxis_change && !xaxis_change))
                                {
                                    y_rangefilteredIndices = filteredY.map((y, index) => y >= yrange[0] && y <= yrange[1] ? index : -1).filter(index => index !== -1);
                                    filteredY = y_rangefilteredIndices.map(index => filteredY[index]);
                                }

                                act_min = Math.min.apply(null, filteredY);
                                act_max = Math.max.apply(null, filteredY);

                                g_ymax = Math.max(g_ymax, act_max);
                                g_ymin = Math.min(g_ymin, act_min);
                            });

                            offset = 1/9*(g_ymax - g_ymin)/2;
                            if(offset == 0)
                            {
                                offset=0.001;
                            }

                            if(i == 0)
                            {
                                ylabel='yaxis.range';
                            }
                            else
                            {
                                ylabel='yaxis'+(1+i)+'.range';
                            }

                            update[ylabel] = [g_ymin-offset,g_ymax+offset];
                        });
                        
                        Plotly.relayout(graphDiv, update);
                    }
                    else
                    {
                        on_event=true;
                    }
                    }

                graphDiv.on('plotly_relayout', test);
            }
        }
        setTimeout(autoscale_function, rec_time);
        '''

        py.plot(fig, filename=file_name, include_mathjax='cdn', auto_open=False) # , include_plotlyjs='cdn'
        with open(file_name, 'r', encoding='utf-8') as file:
            html_content = file.read()
            soup = BeautifulSoup(html_content, 'html.parser')
            first_script_tag = soup.find('script')
            if first_script_tag:
                new_script = soup.new_tag('script')
                new_script.string = "document.body.style.background='#1e1e1e';"+autoscale_code
                first_script_tag.insert_after(new_script)
                with open(file_name, 'w', encoding='utf-8') as file:
                    file.write(str(soup))
        webbrowser.open('file://' + file_name)

###################### MAIN ######################
# Laden der .mat-Datei
# folderpath = "/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/mails/240829_meeting/traj6_poly_shouldertry1/CT_mit_singreg"
# folderpath = "/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/mails/240829_meeting/traj6_poly_shouldertry1/MPC_v1_dyn_transl_schwach_gew"
# folderpath = "/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/mails/240829_meeting/traj6_poly_shouldertry1/MPC_v1_dyn"
folderpath = "/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/mails/240829_meeting/traj6_poly_shouldertry1/MPC_v3_kin_int"
# folderpath = "/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/mails/240829_meeting/traj6_poly_shouldertry1/MPC_v3_kin_int_laenger_praed"

# mat_file_name = '240826_messung11_traj6_ct.mat';
# mat_file_name = '240826_messung12_traj6_mpc1_dyn.mat';
# mat_file_name = '240826_messung13_traj6_mpc1_dyn_transl_schwach_gew.mat';
mat_file_name = '240826_messung14_traj6_mpc8_kin_int.mat';
# mat_file_name = '240826_messung15_traj6_mpc8_kin_int_laenger_praed.mat';

mat_file_path = os.path.join(folderpath, mat_file_name)
outputname = mat_file_name[:-4] + '.html'
output_file_path = os.path.join(folderpath, outputname)

data = sio.loadmat(mat_file_path)

subplot_data = data['signals']

# plot_solution(subplot_data)
plot_solution(subplot_data, plot_fig = False, save_plot=True, file_name=output_file_path)