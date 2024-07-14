#!/bin/bash

# Überprüfen, ob genau zwei Parameter übergeben wurden
if [ "$#" -ne 3 ]; then
  echo "Es müssen drei Parameter angegeben werden: ./replace_script.sh variant solver MPC_index"
  echo ""
  echo "Beispiel 1: ./replace_script.sh opti qrqp 1"
  echo "Beispiel 2: ./replace_script.sh nlpsol ipopt 1"
  echo "Beispiel 3: ./replace_script.sh nlpsol qrqp 1"
  echo ""
  echo "Achtung: Bisher ist nur offline MPC implementiert. Sollte aber auch für online MPC klappen,"
  echo "dazu muss noch ein passendes shared subsystem erstellt werden."
  exit 1
fi

variant=$1
solver=$2
endindex=$3
output="MPC${endindex}_${variant}.slx"
#output="${input//.slx/\_MCP$endindex\_$variant\_$solver.slx}"

# Überprüfen, ob die Eingabedatei existiert
#if [ ! -f "$input" ]; then
#  echo "The input data \"$input\" does not exist."
#  exit 1
#fi

# Überprüfen, ob der variant eine gültige Wahl ist
if [[ $variant =~ "opti" ]] && [[ $variant =~ "nlpsol" ]]; then
  echo "variant can be \"opti\" or \"nlpsol\"."
  echo "chosen variant = $variant."
  exit 2
fi

# Überprüfen, ob der solver für variante opti eine gültige Wahl ist
if ( ! [[ $solver == "qrqp" ]] && ! [[ $solver == "qpoases" ]]) && ! [[ $solver == "ipopt" ]]; then
  echo "solver can be \"qrqp\" or \"ipopt\" or \"qpoases\""
  echo "chosen solver=$solver."
  exit 3
fi

# BEI IF AUF ABSTÄNDE ACHTEN SONST BEKOMMT MAN SYNTAX ERRORS!!!!
# Überprüfen, ob der solver für variante nlpsol eine gültige Wahl ist
if [[ $variant == "nlpsol" ]] && ( ! [[ $solver == "ipopt" ]] && ! [[ $solver == "qpoases" ]] && ! [[ $solver == "qrqp" ]] ); then
  echo "Error: solver=\"$solver\" is not compatible with variant=\"$variant\""
  exit 4
fi

# Überprüfen, ob der solver für variante opti eine gültige Wahl ist
if [[ $variant == "opti" ]] && ! [[ $solver == "qrqp" ]] && ! [[ $solver == "qpoases" ]] ; then
  echo "Error: solver=\"$solver\" is not compatible with variant=\"$variant\""
  exit 4
fi

# Überprüfen, ob der Endindex eine gültige Zahl ist
if ! [[ $endindex =~ ^[0-9]+$ ]]; then
  echo "The new MPC index should be a valid number."
  echo "chosen number is \"$endindex\""
  exit 6
fi

# Kopieren der Datei mit dem entsprechenden Dateinamen
if [[ $variant == "opti" ]]; then
   input="./templates/template_offline_MCP_s_fun_opti_shared_subsystem.slx"
   MPC_string="MPC2"
   solver_string="qrqp"
   idx_val=3
else
   input="./templates/template_offline_MCP_s_fun_nlpsol_shared_subsystem.slx"
   MPC_string="MPC8"
   solver_string="ipopt"
   idx_val=9
fi
tmp_dir="tmp_slx"
mkdir $tmp_dir
unzip $input -d $tmp_dir > /dev/null
#exit 12312321
#cp "$input" "tmp_slx.slx"

# Suchen und Ersetzen des Strings in der kopierten Datei
grep -irn 
find  tmp_slx/ -type f -name "*.xml" -exec sed -i "s/<P Name=\"Value\">$idx_val<\/P>/<P Name=\"Value\">$((endindex+1))<\/P>/g; s/$MPC_string/MPC$endindex/g" {} +

pushd $tmp_dir > /dev/null # shell cd $tmp_dir
zip -r ../$output * > /dev/null
popd  > /dev/null # shell cd ..
rm -rf $tmp_dir

out_dir="./MPC_shared_subsystems/"
mv $output $out_dir
echo "${out_dir}${output} successfully created"
echo ""
echo "use \"close_system('$output')\" in Matlab to reload the model."
echo ""
echo "add the following lines to \"init_MPC_weights.m\":"
echo ""
weight_data="%%%%%%%%%%%%%%%%%%%%%%%%%% MPC1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
MPC='MPC1';
param_weight.(MPC).Q_y      = 1e5*diag([1 1]);   % d_kpn
param_weight.(MPC).Q_y_p    = 1e-5*diag([1 1]); % d_kpn
param_weight.(MPC).Q_y_pp   = 1e-5*diag([1 1]); % d_kpn
param_weight.(MPC).Q_y0_pp  = 1e-5*diag([1 1]); % D_0
param_weight.(MPC).Q_y1     = 1e5*diag([1 1]);   % D_1
param_weight.(MPC).Q_y1_p   = 1e-5*diag([1 1]); % D_1
param_weight.(MPC).Q_y1_pp  = 1e-5*diag([1 1]); % D_1
param_weight.(MPC).Q_yN     = 1e5*diag([1 1]);   % D_N
param_weight.(MPC).Q_yN_p   = 1e-5*diag([1 1]); % D_N

param_weight.(MPC).Q_q_p    = 0*diag([1 1]);  % c_kpn
param_weight.(MPC).Q_q_pp   = 0*diag([1 1]);  % c_kpn
param_weight.(MPC).Q_q0_pp  = 0*diag([1 1]);  % C_0
param_weight.(MPC).Q_qN_p   = 0*diag([1 1]);  % C_N

param_weight.(MPC).x_min    = inf*[ -1; -1; -1; -1 ];
param_weight.(MPC).x_max    = inf*[ 1; 1; 1; 1];
param_weight.(MPC).u_min    = inf*[ -10; -10 ];
param_weight.(MPC).u_max    = inf*[ 10; 10 ];"
weight_data=$(echo "$weight_data" | sed "s/MPC1/MPC$endindex/g")
echo "$weight_data"



#zip -FF "tmp_slx.slx" --out $output # repair corrupted slx. instead of cp, but risky and buggy
