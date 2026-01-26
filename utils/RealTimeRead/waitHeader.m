function waitHeader(s)
    while true
        if s.NumBytesAvailable > 0
            b1 = read(s,1,"uint8");
            if b1 == hex2dec('AA')
                b2 = read(s,1,"uint8");
                if b2 == hex2dec('55')
                    return
                end
            end
        end
    end
end
