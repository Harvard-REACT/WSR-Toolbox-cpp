function viz_profile(profile)
    beta_min = -180*(pi/180);
    beta_max = 180*(pi/180);
    gamma_min = 0*(pi/180);
    gamma_max = 180*(pi/180);       
    nbeta = 360;
    ngamma = 180;
    betaList = linspace(beta_min, beta_max, nbeta).';
    gammaList = linspace(gamma_min, gamma_max, ngamma);
    beta_profile_csv = readtable(profile);
    beta_profile_cpp = table2array(beta_profile_csv);
    figure(12233);
    %figure; clf; hold on;
    surf(betaList*180/pi, gammaList*180/pi, beta_profile_cpp.', 'EdgeColor', 'none');
    xlabel('Beta');
    ylabel('Gamma');
end