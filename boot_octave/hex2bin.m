function [data] = hex2bin(fpath)
% Simple and not effective loader for Intel HEX data
% (c) 2020, Stanislav Maslan, s.maslan@seznam.cz
% MIT License

    % initialize data array and size
    data = repmat(uint8(0xFF),[65536 1]);
    len = 0;
    
    % load file
    fr = fopen(fpath,'r');  
    
    while(1)
        % get row
        str = fgets(fr);
        if isempty(str)
           break; 
        end
        % get record data size
        rsize = hex2dec(str(2:3));
        % address
        raddr = hex2dec(str(4:7));
        % get record type
        rtype = hex2dec(str(8:9));
        if rtype == 0x00
            % code data
            row = hex2dec(reshape(str(10:10+2*rsize-1),[2 rsize])');    
            % total row HEX size
            hsize = rsize + 1 + 2 + 1;    
            % row CRC value
            rcrc = hex2dec(str(2+hsize*2:3+hsize*2));
            % calculate CRC
            crc = mod(256-sum(hex2dec(reshape(str(2:1+hsize*2),[2 hsize])')),256);
            if rcrc ~= crc
                error('Loading Intel HEX failed at address %04X: invalid CRC!',raddr);
            end        
            % store data to bufer
            data(1+raddr:raddr+rsize,1) = row;
            % store maximum address
            len = max(len,raddr + rsize);    
        elseif rtype == 0x01
            % end of file
            break;
        end
    end
    
    % get rid of unused data array
    data = data(1:len);
    
    % close file
    fclose(fr);
    
end