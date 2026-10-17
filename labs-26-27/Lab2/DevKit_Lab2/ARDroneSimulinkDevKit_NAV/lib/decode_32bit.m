function res = decode_32bit(input, mark)
% if mark = 1; then output a int
% else output the float value which is defined by SDK description.
% the input is an array (1*4)
% e.g. : NDS(Battery_OS:Battery_OS+data_length-1)

hex_value = dec2hex(input);
hex_value = [hex_value(4,:),hex_value(3,:),hex_value(2,:),hex_value(1,:)];
if mark ==1
    res = typecast(input,'uint32');%hex2dec(hex_value);
elseif mark ==2
    res = hex2dec(hex_value)-2^31;
else
    res = typecast(input,'single');%typecast(uint32(hex2dec(hex_value)),'single');%
end
res = res(1);
end
