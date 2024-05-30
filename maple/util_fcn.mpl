file_export_C := proc (sym:: ~Matrix, name::string, 
parameterlist::list, folder_name::string, abs_file_path::string, type::string := ".hpp") local s, m, tmp, f, i, rows, 
columns, param, abs_dir_path; s := convert(indets(sym,'symbol'),list); for i to
numelems(parameterlist) do s := subs(parameterlist[i] = NULL,s); end do; rows,
columns := LinearAlgebra[Dimension](sym); if columns < rows then m := convert(
LinearAlgebra[Transpose](sym),array); else m := convert(sym,array); end if; if
0 < numelems(s) then tmp := codegen[makeproc](m,parameters = [op(parameterlist)
, param]); else tmp := codegen[makeproc](m,parameters = parameterlist); end if;
tmp := codegen[optimize](tmp); tmp := CodeGeneration[C](tmp,output = string,
optimize = false,deducetypes = false,defaulttype = float); for i to numelems(s)
do tmp := StringTools[RegSubs](cat("([ \\(\\-])(",convert(s[i],string),
")([; \\(\\)])") = "\\1param.\\2\\3",tmp); end do; tmp := StringTools[
SubstituteAll](tmp," double param,","const RobotParams& param,"); if type = 
".hpp" then tmp := StringTools[SubstituteAll](tmp,"#include <math.h>",""); tmp
:= StringTools[SubstituteAll](tmp,"void tmp (",cat(
"template<typename Scalar> void ",name," (")); tmp := StringTools[SubstituteAll
](tmp,"double ","Scalar "); end if; if columns < rows then tmp := StringTools[
SubstituteAll](tmp,"tmpreturn = m;","tmpreturn = m';"); end if; abs_dir_path :=
cat(abs_file_path,"/cpp/",folder_name,"/"); if FileTools[
Exists](abs_dir_path) = false then mkdir(abs_dir_path); end if; f := FileTools[
Text][Open](cat(abs_dir_path, "/", name,type),create,overwrite); FileTools[Text][
WriteString](f,tmp); FileTools[Text][Close](f); end proc;
file_export := proc (sym:: ~Matrix, name::string, 
parameterlist::list, folder_name::string, abs_file_path::string) local s, m, tmp, f, i, rows, columns, param, abs_dir_path;
s := convert(indets(sym,'symbol'),list); for i to numelems(parameterlist) do s
:= subs(parameterlist[i] = NULL,s); end do; rows, columns := LinearAlgebra[
Dimension](sym); if columns < rows then m := convert(LinearAlgebra[Transpose](
sym),array); else m := convert(sym,array); end if; if 0 < numelems(s) then tmp
:= codegen[makeproc](m,parameters = [op(parameterlist), param]); else tmp := 
codegen[makeproc](m,parameters = parameterlist); end if; tmp := codegen[
optimize](tmp); tmp := CodeGeneration[Matlab](tmp,output = string,optimize = 
true); for i to numelems(s) do tmp := StringTools[RegSubs](cat("([ \\(\\-])(",
convert(s[i],string),")([; \\(\\)])") = "\\1param.\\2\\3",tmp); end do; tmp :=
StringTools[SubstituteAll](tmp,"function tmpreturn = tmp(",cat(
"function tmpreturn = ",name,"(")); if columns < rows then tmp := StringTools[
SubstituteAll](tmp,"tmpreturn = m;","tmpreturn = m';"); end if; abs_dir_path :=
cat(abs_file_path,"/", folder_name,"/"); if 
FileTools[Exists](abs_dir_path) = false then mkdir(abs_dir_path); end if; f :=
FileTools[Text][Open](cat(abs_dir_path,"/",name,".m"),create,overwrite); FileTools[
Text][WriteString](f,tmp); FileTools[Text][Close](f); end proc;
