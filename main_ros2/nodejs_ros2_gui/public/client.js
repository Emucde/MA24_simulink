var ws;//aktuelle websocket sitzung

window.onload = start;

function start() {
	//ZEIT aktualisieren:
	document.getElementById("timeconn").innerHTML=new Date().getTime();

        try{
            ws = new WebSocket('ws://localhost:8080');
        }
        catch(e){
            console.log("error ",e);
        }


    ws.onopen = function(e){
      console.log("NodeJs connected");
      document.getElementById("connected").innerHTML = "NodeJs connected @ ";
      // Format the date nicely
      const formattedDate = new Date().toLocaleString('en-US', { 
          weekday: 'long', 
          year: 'numeric', 
          month: 'long', 
          day: 'numeric', 
          hour: '2-digit', 
          minute: '2-digit', 
          second: '2-digit'
      });
      document.getElementById("timeconn").innerHTML = formattedDate; // Display the formatted date
      document.getElementById("conncon").style.backgroundColor = "green";
    }

    ws.onclose = function(){
        console.log("NodeJs disconnected");
        //bringt da nix:
        //datarunning = 0;//falls repetetive messung laeuft.
        document.getElementById("connected").innerHTML="NodeJs disconnected @";
        document.getElementById("timeconn").innerHTML=new Date();
        document.getElementById("conncon").style.backgroundColor="red";

        //labview (TCP) ebenfalls disconnected
        document.getElementById("labconncon").style.backgroundColor="red";
        document.getElementById("labcon").innerHTML="Labview disconnected @ " + new Date();

        //Reconnect every 5000 seconds
        setTimeout(function(){start()}, 5000);
    }

    ws.onmessage = (event) => {
        var data;
        try{
           data = JSON.parse(event.data);
        }
        catch(e){
          data = event.data;
        }
        const responseElement = document.getElementById('response');
        if (data.status === 'success') {
            result = data.result;
            if (result.name == "traj_names") {
                /*
                add trajectories to the dropdown menu
                <select id="trajectory">
                    <option value="1">traj1</option>
                    <option value="2">traj2</option>
                </select>
                */
                const trajSelect = document.getElementById('trajectory');
                trajSelect.innerHTML = '';
                for (let i = 0; i < result.trajectories.length; i++) {
                    const option = document.createElement('option');
                    option.value = i+1; // starts at 1 !!
                    option.textContent = result.trajectories[i];
                    trajSelect.appendChild(option);
                }
                var a_iframe_plot = document.querySelector('a[_target=iframe_plot1]')
                a_iframe_plot.textContent = result.trajectories[0];
                change_button_positions();
            }
            else{
                responseElement.textContent = `${result.name}: ${result.status}`;
                if(result.name == "ros_service"){
                    var ros_connected_col_el = document.getElementById("labconncon");
                    var ros_connected_text_el = document.getElementById("labcon");
                    if(result.status.includes("timed out")){
                      ros_connected_col_el.style.backgroundColor = "red";
                      ros_connected_text_el.textContent = "ROS2 disconnected";
                    }
                    else {
                      ros_connected_col_el.style.backgroundColor = "green";
                      ros_connected_text_el.textContent = "ROS2 connected";
                    }
                    if(result.status.includes("homing in progress"))
                    {
                        document.getElementById("home_main").classList.remove("hide");
                    }
                    if(result.status.includes("homing done"))
                    {
                        document.getElementById("home_main").classList.add("hide");
                    }
                }
            }
        } else {
            responseElement.textContent = `Error: ${data.error}`;
        }
    };
}

document.getElementById('start').addEventListener('click', () => {
    ws.send(JSON.stringify({ command: 'start' }));
});

document.getElementById('reset').addEventListener('click', () => {
    ws.send(JSON.stringify({ command: 'reset' }));
});

document.getElementById('stop').addEventListener('click', () => {
    ws.send(JSON.stringify({ command: 'stop' }));
});

document.getElementById('home').addEventListener('click', () => {
    var home_delay = document.querySelector('#home_delay').value;
    ws.send(JSON.stringify({ command: 'home', delay: home_delay }));
});

document.getElementById('open_brakes').addEventListener('click', () => {
  ws.send(JSON.stringify({ command: 'open_brakes' }));
});

document.getElementById('close_brakes').addEventListener('click', () => {
  ws.send(JSON.stringify({ command: 'close_brakes' }));
});

const toggle = document.getElementById('toggle');
const body = document.body;

toggle.addEventListener('change', () => {
    if (toggle.checked) {
        body.classList.add('light-theme');
        body.classList.remove('dark-theme');
    } else {
        body.classList.remove('light-theme');
        body.classList.add('dark-theme');
    }
});

if (localStorage.getItem('theme') === 'light') {
    toggle.checked = true;
    body.classList.add('light-theme');
} else {
    body.classList.remove('light-theme');
}

toggle.addEventListener('change', () => {
    localStorage.setItem('theme', toggle.checked ? 'light' : 'dark');
});

function send_trajectory(current_element) {
    var traj_num = parseInt(current_element.value, 10);
    var traj_text = current_element.options[traj_num-1].text;
    ws.send(JSON.stringify({ command: 'trajectory_selection', traj_select: traj_num }));
    var a_iframe_plot = document.querySelector('a[_target=iframe_plot1]')
    a_iframe_plot.textContent = traj_text;
    change_button_positions();
}

  
function click_fun(element, number) {
    var main_div = element.parentNode.parentNode;
    var plot_iframe = main_div.querySelectorAll('iframe')[number];
    plot_iframe.style.display="block";

    // always reload iframe
    element.target = element.getAttribute('_target');
    element.href = element.getAttribute('_href');
    element.setAttribute('is_loaded', 1);
    if(number == 0)
    {
        button_tag = main_div.querySelector('.hide_plot_button');
        text_hide = 'Hide Plot';
    }
    else
    {
        button_tag = main_div.querySelector('.hide_video_button');
        text_hide = 'Hide Visualize';
    }
    button_tag.textContent = text_hide;
    plot_iframe.setAttribute('is_shown', 1);
  }
  
  function hide_iframe(element, number) {
    var main_div = element.parentNode.parentNode;
    var plot_iframe = main_div.querySelectorAll('iframe')[number];
    var is_shown = plot_iframe.getAttribute('is_shown');

    var prev_a_tag;
    var text_show;
    var text_hide;
    if(number == 0)
    {
        prev_a_tag = main_div.querySelector('.plots_a');
        text_show = 'Show Plot';
        text_hide = 'Hide Plot';
    }
    else
    {
        prev_a_tag = main_div.querySelector('.visualize_a');
        text_show = 'Show Visualize';
        text_hide = 'Hide Visualize';
    }
    var is_loaded = prev_a_tag.getAttribute('is_loaded');

    if(is_shown == "1")
    {
      plot_iframe.style.display="none";
      plot_iframe.setAttribute('is_shown', 0);
      element.textContent = text_show;
    }
    else
    {
      plot_iframe.style.display="block";
      plot_iframe.setAttribute('is_shown', 1);
      element.textContent = text_hide;

      if(is_loaded == "0")
      {
        prev_a_tag.click();
      }
    }
  }
  
  function hide_settings(element) {
    var main_div = element.parentNode.parentNode;
    var next_code_area = main_div.querySelector('pre')
    var is_shown = next_code_area.getAttribute('is_shown');
    
    if(is_shown == "1")
    {
      next_code_area.style.display="none";
      next_code_area.setAttribute('is_shown', 0);
      element.textContent = 'Show Settings';
    }
    else
    {
      next_code_area.style.display="block";
      next_code_area.setAttribute('is_shown', 1);
      element.textContent = 'Hide Settings';
    }
  }

  function load_all_data() {
    var a_tags = document.querySelectorAll('a');
    a_tags.forEach(function(el, i){
      window.setTimeout(function(){
        el.click();
        window.setTimeout(function(){
          el.nextElementSibling.click();
        }, 500);
      }, 2000 * i);
    });
  }

  function change_video_size(element) {

    var act_size = element.getAttribute('act_size');
    var videos = document.querySelectorAll('video');
    var new_size = act_size == "small" ? "big" : "small";
    var new_width = new_size == "small" ? "320px" : "600px";
    element.setAttribute('act_size', new_size);
    element.innerHTML = new_size == "small" ? "Video size 320 px" : "Video size 600 px";

    videos.forEach(function(el, i){
      el.style.width = new_width;
    });
  }

  function autoplay(element) {
    var is_on = element.getAttribute('is_on');
    var new_state = is_on == "1" ? "0" : "1";
    element.setAttribute('is_on', new_state);
    element.innerHTML = new_state == "1" ? "Autoplay ON" : "Autoplay OFF";
  }

  function change_button_positions() {
    console.log(window.innerHeight);
    console.log(window.innerWidth);
    var width_arr = [];
    document.querySelectorAll('.plots_a').forEach(function(el, i){
      width_arr.push(el.offsetWidth + 100);
    });
    max_width = Math.max.apply(null, width_arr);

    var offset = 50;

    document.querySelectorAll('.hide_plot_button').forEach(function(el, i){
      el.style.left = max_width + offset + 'px';
      el.style.display = 'initial';
    });

    document.querySelectorAll('.hide_settings_button').forEach(function(el, i){
      el.style.left = max_width + offset + 95 + 'px';
      el.style.display = 'initial';
    });

    document.querySelectorAll('.hide_video_button').forEach(function(el, i){
      el.style.left = max_width + offset + 215 + 'px';
      el.style.display = 'initial';
    });
    console.log(document.querySelectorAll('.hide_video_button'))
  }
