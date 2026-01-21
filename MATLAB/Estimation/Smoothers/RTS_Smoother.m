classdef RTS_Smoother < handle

    properties
        x_k
        K_k
    end

    methods
        function obj = RTS_Smoother(x_f_all, P_prior_f_all, P_post_f_all, Phi_f_all, num_steps)
            
            x_k_all = zeros(num_steps, width(x_f_all));
            
            for k=num_steps:-1:2
                P_f_k_plus = squeeze(P_prior_f_all(:,:,k));
                P_f_k_min = squeeze(P_post_f_all(:,:,k));

                I = inv(P_f_k_plus);
                K_k = 
            end

        end
    end

end