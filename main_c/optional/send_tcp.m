clc;
clear;
close all;
%send tcp

echotcpip("off")
clear t
t = tcpclient("127.0.0.1",1337);

configureCallback(t,"byte",26,@send_data)

%write(t,data);
%write(t,data*10);

%pause(1)
%clear t
%echotcpip("off")

function send_data(t, event)
    fprintf("empfangen: %s\n", t.read())
    fprintf("send data\n")
    N=5000;
    i_arr=linspace(0,1,N);
    data=2.5*cos(2*pi*i_arr)+0.1*rand(1,5000);

    write(t,'0'); %tcp client sagen, dass daten kommen
    write(t,data);
    write(t,data*10);
end