# This script creates an HTML index file for a directory structure containing data files.
# It generates a structured HTML page with links to data files, settings, and videos.

import os
import re
from bs4 import BeautifulSoup

def find_bottom_directories(base_path):
    bottom_dirs = []
    for root, dirs, files in os.walk(base_path):
        if not dirs:  # Wenn keine Unterverzeichnisse vorhanden sind
            bottom_dirs.append(root)
    return bottom_dirs

def group_bottom_directories(base_path):
    bottom_dirs = find_bottom_directories(base_path)
    grouped_dirs = {}

    for dir_path in bottom_dirs:
        # Finde den direkten übergeordneten Ordner
        parent_path = os.path.dirname(dir_path)
        
        # Überprüfe, ob der übergeordnete Ordner der Basisordner ist
        if parent_path == base_path:
            continue  # Überspringe Ordner, die direkt im Basisordner sind
        
        # Gruppiere die Verzeichnisse nach ihrem übergeordneten Ordner
        if parent_path not in grouped_dirs:
            grouped_dirs[parent_path] = []
        grouped_dirs[parent_path].append(dir_path)

    return grouped_dirs

def search_files_in_bottom_dirs(base_path, file_pattern='*'):
    bottom_dirs = find_bottom_directories(base_path)
    results = {}
    for dir_path in bottom_dirs:
        matching_files = [f for f in os.listdir(dir_path) if file_pattern in f and os.path.isfile(os.path.join(dir_path, f))]
        if matching_files:
            results[dir_path] = matching_files
    return results

def get_link_text(dir_path, file_name):
    # Suche nach filename.txt und lese den Inhalt
    txt_file_path = os.path.join(dir_path, 'filename.txt')
    if os.path.exists(txt_file_path):
        with open(txt_file_path, 'r', encoding='utf-8') as f:
            return f.read().strip()
    else:
      return file_name
    
def read_text_file(dir_path, file_name):
    # Suche nach filename.txt und lese den Inhalt
    txt_file_path = os.path.join(dir_path, file_name)
    if os.path.exists(txt_file_path):
        with open(txt_file_path, 'r', encoding='utf-8') as f:
            return f.read().strip()
    else:
      return file_name + ' not found!'

def get_mp4_file(path):
    for root, dirs, files in os.walk(path):
        for file in files:
            if file.endswith('.mp4'):
              return file
    return 'no mp4 file found!'

def create_html_structure(base_path, title_text='index.html'):
    # HTML-Grundgerüst
    html_template = """
<!DOCTYPE html>
<html>
<head>
    <title>index.html</title>
    <script src="https://cdn.rawgit.com/google/code-prettify/master/loader/run_prettify.js?lang=matlab"></script>

    <style>
    /* Pretty printing styles. Used with prettify.js. */
    /* Vim sunburst theme by David Leibovic */

    pre .str, code .str { color: #65B042; } /* string  - green */
    pre .kwd, code .kwd { color: #E28964; } /* keyword - dark pink */
    pre .com, code .com { color: #74d181; font-style: italic; } /* comment - gray */
    pre .typ, code .typ { color: #89bdff; } /* type - light blue */
    pre .lit, code .lit { color: #3387CC; } /* literal - blue */
    pre .pun, code .pun { color: #fff; } /* punctuation - white */
    pre .pln, code .pln { color: #fff; } /* plaintext - white */
    pre .tag, code .tag { color: #89bdff; } /* html/xml tag    - light blue */
    pre .atn, code .atn { color: #bdb76b; } /* html/xml attribute name  - khaki */
    pre .atv, code .atv { color: #65B042; } /* html/xml attribute value - green */
    pre .dec, code .dec { color: #3387CC; } /* decimal - blue */
    pre .ident {color : #fff;} /* white */
    pre .const { color: #3387CC; } /* literal - blue */

    pre.prettyprint, code.prettyprint {
      background-color: #151b23;
      border-radius: 8px;
    }

    pre.prettyprint {
      width: 95%;
      margin: 1em auto;
      padding: 1em;
      white-space: pre-wrap;
    }


    /* Specify class=linenums on a pre to get line numbering */
    ol.linenums { margin-top: 0; margin-bottom: 0; color: #AEAEAE; } /* IE indents via margin-left */
    li.L0,li.L1,li.L2,li.L3,li.L5,li.L6,li.L7,li.L8 { list-style-type: none }
    /* Alternate shading for lines */
    li.L1,li.L3,li.L5,li.L7,li.L9 { }

    @media print {
      pre .str, code .str { color: #060; }
      pre .kwd, code .kwd { color: #006; font-weight: bold; }
      pre .com, code .com { color: #600; font-style: italic; }
      pre .typ, code .typ { color: #404; font-weight: bold; }
      pre .lit, code .lit { color: #044; }
      pre .pun, code .pun { color: #440; }
      pre .pln, code .pln { color: #000; }
      pre .tag, code .tag { color: #006; font-weight: bold; }
      pre .atn, code .atn { color: #404; }
      pre .atv, code .atv { color: #060; }
    }
    </style>

    <style>
    h1, h2 {color:white;}
    p {color:white; margin-bottom:8px;}
    title {color:white;}
    a {color:red;}
    li {color:white;margin-bottom:0px;}
    hr {  margin-top: 28px;
          width: 25%;
          margin-left: 0;}
      iframe {    display: block;       /* iframes are inline by default */
    background: #000;
    border: none;         /* Reset default border */
    height: 1099px;        /* Viewport-relative units */
    width: 100%;}
    button {margin-left: 10px;}
    </style>
    <script>  
    function click_fun(element) {
      var is_loaded = element.getAttribute('is_loaded');
      var next_iframe = element.parentNode.nextElementSibling.nextElementSibling.nextElementSibling;
      next_iframe.style.display="block";

      // always reload iframe
      element.target = element.getAttribute('_target');
      element.href = element.getAttribute('_href');
      element.setAttribute('is_loaded', 1);
      element.nextSibling.textContent = 'Hide Plot';
      next_iframe.setAttribute('is_shown', 1);
    }
    
    function hide_plot(element) {
      var next_iframe = element.parentNode.nextElementSibling.nextElementSibling.nextElementSibling;
      var is_shown = next_iframe.getAttribute('is_shown');
      var prev_a_tag = element.previousSibling;
      var is_loaded = prev_a_tag.getAttribute('is_loaded');
      
      if(is_shown == "1")
      {
        next_iframe.style.display="none";
        next_iframe.setAttribute('is_shown', 0);
        element.textContent = 'Show Plot';
      }
      else
      {
        next_iframe.style.display="block";
        next_iframe.setAttribute('is_shown', 1);
        element.textContent = 'Hide Plot';

        if(is_loaded == "0")
        {
          prev_a_tag.click();
        }
      }
    }
    
    function hide_settings(element) {
      var next_code_area = element.parentNode.nextElementSibling.nextElementSibling;
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

    function hide_video(element) {
      var next_video = element.parentNode.nextElementSibling;
      var is_shown = next_video.getAttribute('is_shown');
      
      if(is_shown == "1")
      {
        next_video.style.display="none";
        next_video.setAttribute('is_shown', 0);
        element.textContent = 'Show Video';
      }
      else
      {
        next_video.style.display="block";
        next_video.setAttribute('is_shown', 1);
        var autoplay_button = document.getElementById('autoplay_button');
        var is_on = autoplay_button.getAttribute('is_on');
        if(is_on == "1")
        {
          next_video.currentTime=0;
          next_video.play();
        }
        element.textContent = 'Hide Video';
      }
    }

    function load_all_data() {
      var a_tags = document.querySelectorAll('a');
      a_tags.forEach(function(el, i){
        window.setTimeout(function(){
          el.click();
          window.setTimeout(function(){
            el.nextSibling.click();
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
      document.querySelectorAll('a').forEach(function(el, i){
        width_arr.push(el.offsetWidth);
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

    window.onload = change_button_positions;
    </script>
</head>
<body style="background-color:#1e1e1e;">
</body>
</html>
"""

    # Parse das HTML-Template
    soup = BeautifulSoup(html_template, 'html.parser')

    title_path = os.path.join(base_path, 'main_title.txt')
    if os.path.exists(title_path):
        with open(title_path, 'r', encoding='utf-8') as f:
            main_title = f.read().strip()
            title = soup.find('title')
            title.string = os.path.basename(main_title)

    body = soup.body

    header_path = os.path.join(base_path, 'main_header.txt')
    if os.path.exists(header_path):
        with open(header_path, 'r', encoding='utf-8') as f:
            main_header = f.read().strip()

    # Füge h2 Header hinzu
    h1 = soup.new_tag('h1')
    h1.string = main_header
    body.append(h1)

    btn_div = soup.new_tag('div')

    load_all_button = soup.new_tag('button', onclick="load_all_data();")
    load_all_button.string = "Load All Plots"

    btn_div.append(load_all_button)

    vid_size_button = soup.new_tag('button', onclick="change_video_size(this);")
    vid_size_button.string = "Video size 600 px"
    vid_size_button['act_size'] = "big"

    btn_div.append(vid_size_button)

    autoplay_button = soup.new_tag('button', onclick="autoplay(this);", id="autoplay_button")
    autoplay_button.string = "Autoplay ON"
    autoplay_button['is_on'] = "1"

    btn_div.append(autoplay_button)

    body.append(btn_div)


    iframe_counter = 1

    grouped_dirs = group_bottom_directories(base_path)

    for parent_dir, bottom_dirs in grouped_dirs.items():
      # Lese den Header aus der header.txt im übergeordneten Ordner
      header_path = os.path.join(parent_dir, 'header.txt')
      if os.path.exists(header_path):
          with open(header_path, 'r', encoding='utf-8') as f:
              header = f.read().strip()
      else:
          header = os.path.basename(parent_dir)

      hr = soup.new_tag('hr')
      body.append(hr)

      # Füge h2 Header hinzu
      h2 = soup.new_tag('h2')
      h2.string = header
      body.append(h2)

      for dir_path in bottom_dirs:
          for file in sorted(os.listdir(dir_path)):
              if file.endswith('.html'):
                  file_name = os.path.splitext(file)[0]
                  file_path = os.path.relpath(os.path.join(dir_path, file), base_path)
                  
                  link_text = get_link_text(dir_path, file)

                  # Erstelle li Element
                  li = soup.new_tag('li')
                  
                  # Erstelle a Element
                  a = soup.new_tag('a', href="javascript:void(0)", target="", onclick="click_fun(this)")
                  a['is_loaded'] = "0"
                  a['_target'] = f"iframe{iframe_counter}"
                  a['_href'] = file_path
                  a.string = link_text
                  li.append(a)

                  hide_plot_button = soup.new_tag('button', onclick="hide_plot(this);", style="display:none; position: absolute; left: 725px;")
                  hide_plot_button['class'] = 'hide_plot_button'
                  hide_plot_button.string = "Show Plot"

                  li.append(hide_plot_button)

                  settings_button = soup.new_tag('button', onclick="hide_settings(this);", style="display:none; position: absolute; left: 820px;")
                  settings_button['class'] = 'hide_settings_button'
                  settings_button.string = "Show Settings"

                  li.append(settings_button)

                  video_button = soup.new_tag('button', onclick="hide_video(this);", style="display:none; position: absolute; left: 940px;")
                  video_button['class'] = 'hide_video_button'
                  video_button.string = "Show Video"

                  li.append(video_button)

                  body.append(li)

                  video = soup.new_tag('video', width="600px", controls="", style="display:none;margin-left:22px;margin-top:8px;")
                  video['is_shown'] = "0"
                  source = soup.new_tag('source', src=os.path.join(dir_path, get_mp4_file(dir_path)), type="video/mp4")
                  source.string = "Your browser does not support the video tag."
                  video.append(source)

                  body.append(video)

                  # erstelle settings.txt code area
                  pre = soup.new_tag('pre', style="width: 80%;margin-left:22px;display:none;")
                  pre['class'] = 'prettyprint lang-matlab'
                  pre['is_shown'] = '0'

                  code = soup.new_tag('code')

                  code_text = read_text_file(dir_path, 'settings.txt')
                  code.string = code_text

                  pre.append(code)

                  body.append(pre)

                  # Erstelle iframe Element
                  iframe = soup.new_tag('iframe', src="about:blank", title="description", style="display:none;")
                  iframe['name'] = f"iframe{iframe_counter}"
                  iframe['is_shown'] = "0"
                  body.append(iframe)

                  if(dir_path != bottom_dirs[-1]):
                    body.append(soup.new_tag('br'))

                  iframe_counter += 1

    # Schreibe das Ergebnis in index.html
    with open(os.path.join(base_path, 'index.html'), 'w', encoding='utf-8') as f:
        html_str = str(soup)
        f.write(html_str)

# Verwendung:
base_path = "/home/rslstudent/Students/Emanuel/bespr_29_oct/241023_messungen/"  # Ersetzen Sie dies durch Ihren tatsächlichen Pfad
create_html_structure(base_path)