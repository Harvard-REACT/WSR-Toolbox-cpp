function viz_aoa(filename, nphi, ntheta, phi_min, phi_max,theta_min, theta_max)
    
    nbeta = nphi;
    ngamma = ntheta;      
    beta_min = phi_min*(pi/180);
    beta_max = phi_max*(pi/180);
    gamma_min = theta_min*(pi/180);
    gamma_max = theta_max*(pi/180);      
    betaList = linspace(beta_min, beta_max, nbeta).';
    gammaList = linspace(gamma_min, gamma_max, ngamma);
    beta_profile_csv = readtable(filename);
    beta_profile_cpp = table2array(beta_profile_csv);
    figure(12233);
    subplot(2,1,1)
    surf(betaList*180/pi, gammaList*180/pi, beta_profile_cpp.', 'EdgeColor', 'none');
    set(gcf,'Renderer','Zbuffer')            
    xlabel('Azimuth (Degree)');
    ylabel('Elevation (Degree)');
    title(sprintf('AOA profile (side view)'));         
    subplot(2,1,2)
    surf(betaList*180/pi, gammaList*180/pi, beta_profile_cpp.', 'EdgeColor', 'none');
    set(gcf,'Renderer','Zbuffer');
    view(2)
    title('AOA profile Top View');
    xlabel('Beta');
    ylabel('Gamma');

end
