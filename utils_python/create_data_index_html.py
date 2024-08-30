import os
import re
from bs4 import BeautifulSoup

def find_bottom_directories(base_path):
    bottom_dirs = []
    for root, dirs, files in os.walk(base_path):
        if not dirs:  # Wenn keine Unterverzeichnisse vorhanden sind
            bottom_dirs.append(root)
    return bottom_dirs

def search_files_in_bottom_dirs(base_path, file_pattern='*'):
    bottom_dirs = find_bottom_directories(base_path)
    results = {}
    for dir_path in bottom_dirs:
        matching_files = [f for f in os.listdir(dir_path) if file_pattern in f and os.path.isfile(os.path.join(dir_path, f))]
        if matching_files:
            results[dir_path] = matching_files
    return results

def create_html_structure(base_path):
    # HTML-Grundgerüst
    html_template = """
<!DOCTYPE html>
<html>
<head>
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
    </style>
    <script>  
    function click_fun(element) {
      var is_loaded = element.getAttribute('is_loaded');
      var next_iframe = element.parentNode.nextElementSibling;
      next_iframe.style.display="block";
      if(is_loaded == "0")
      {
        const newButton = document.createElement('button');
        newButton.textContent = 'Hide Plots!';
        newButton.onclick = function(){
          hide_plot(this);
        };
        element.insertAdjacentElement('afterend', newButton);
        element.setAttribute('is_loaded', 1);
       }
    }
    
    function hide_plot(element) {
      var next_iframe = element.parentNode.nextElementSibling;
      var is_shown = next_iframe.getAttribute('is_shown');
      
      if(is_shown == "1")
      {
        next_iframe.style.display="none";
        next_iframe.setAttribute('is_shown', 0);
        element.textContent = 'Show Plots!';
      }
      else
      {
        next_iframe.style.display="block";
        next_iframe.setAttribute('is_shown', 1);
        element.textContent = 'Hide Plots!';
      }
    }
    let iframeCounter = 1;
    </script>
</head>
<body style="background-color:#1e1e1e;">
  <h1> Simulationsergebnisse vom 26.08.2024 </h1>
</body>
</html>
"""

    # Parse das HTML-Template
    soup = BeautifulSoup(html_template, 'html.parser')
    body = soup.body

    iframe_counter = 1

    # Suche nach HTML-Dateien in den untersten Verzeichnissen
    found_files = search_files_in_bottom_dirs(base_path, ".html")

    for dir_path, files in found_files.items():
        # Lese den Header aus der header.txt
        header_path = os.path.join(dir_path, 'header.txt')
        if os.path.exists(header_path):
            with open(header_path, 'r') as f:
                header = f.read().strip()
        else:
            header = os.path.basename(dir_path)  # Fallback, wenn keine header.txt existiert

        # Füge h2 Header hinzu
        h2 = soup.new_tag('h2')
        h2.string = header
        body.append(h2)

        for file in sorted(files):
            file_name = os.path.splitext(file)[0]
            file_path = os.path.relpath(os.path.join(dir_path, file), base_path)
            
            # Extrahiere den Messnamen aus dem Dateinamen
            match = re.search(r'messung(\d+)', file_name, re.IGNORECASE)
            if match:
                messung_nr = match.group(1)
                link_text = f"Messung {messung_nr}:"
            else:
                link_text = file_name

            # Erstelle li Element
            li = soup.new_tag('li')
            
            # Erstelle a Element
            a = soup.new_tag('a', href=file_path, target=f"iframe{iframe_counter}", onclick="click_fun(this)")
            a['is_loaded'] = "0"
            a.string = link_text
            li.append(a)
            body.append(li)

            # Erstelle iframe Element
            iframe = soup.new_tag('iframe', src="about:blank", title="description", style="display:none;")
            iframe['name'] = f"iframe{iframe_counter}"
            iframe['is_shown'] = "1"
            body.append(iframe)

            # Füge einen Zeilenumbruch hinzu
            body.append(soup.new_tag('br'))

            iframe_counter += 1

    # Schreibe das Ergebnis in index.html
    with open('/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/mails/240829_meeting/index.html', 'w', encoding='utf-8') as f:
        f.write(str(soup.prettify()))

# Verwendung:
base_path = "/media/daten/Projekte/Studium/Master/Masterarbeit_SS2024/2DOF_Manipulator/mails/240829_meeting/"  # Ersetzen Sie dies durch Ihren tatsächlichen Pfad
create_html_structure(base_path)