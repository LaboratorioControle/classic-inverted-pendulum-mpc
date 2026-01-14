function print_cpp_matrix(name, M)
    [r, c] = size(M);

    fprintf("%s = {\n", name);

    for i = 1:r
        fprintf("    { ");
        for j = 1:c
            fprintf("%.10f", M(i,j)); 
            if j < c
                fprintf(", ");
            end
        end
        fprintf(" }");
        if i < r
            fprintf(",\n");
        else
            fprintf("\n");
        end
    end

    fprintf("};\n");
end
