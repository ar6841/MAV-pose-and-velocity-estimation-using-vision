function res = getCorner(id)
persistent res_table
if isempty(res_table)
    d=0.152;
    offset=0.178-0.152;
    for i=1:12 
        for j=1:9
            if(j<=3)
                n_offset=0;
            elseif(j>3&&j<7)
                n_offset=1;
            else
                n_offset=2;
            end
            element_num=(j-1)*12+i;
            res_table(1,element_num)=(2*i-2)*d+d/2;
            res_table(2,element_num)=(2*j-2)*d+d/2+n_offset*offset;
            res_table(3,element_num)=(2*i-1)*d;
            res_table(4,element_num)=(2*j-2)*d+n_offset*offset;
            res_table(5,element_num)=(2*i-1)*d;
            res_table(6,element_num)=(2*j-1)*d+n_offset*offset;
            res_table(7,element_num)=(2*i-2)*d;
            res_table(8,element_num)=(2*j-1)*d+n_offset*offset;
            res_table(9,element_num)=(2*i-2)*d;
            res_table(10,element_num)=(2*j-2)*d+n_offset*offset; 
        end
    end
end
res=res_table(:,id+1);
end
