function displayMessage(msg,title)
close;
CreateStruct.Interpreter = 'tex';
CreateStruct.WindowStyle = 'modal';
msg= strcat('\fontsize{10}',msg,'.Click OK to continue.');
dlg =msgbox(msg,title,CreateStruct);
waitfor(dlg);
end

