function check_music
    H_real = readtable('H_real_1804289383.csv');
    H_imag = readtable('H_imag_1804289383.csv');
    e_term = readtable('e_term_1804289383.csv');
    lambda_inv = 18.4677777777778;
    e_term_exp = exp((double(e_term{:,:}).*(-1i*4*pi*lambda_inv)));
    
    H = H_real{:,:} + 1j*H_imag{:,:};
    
%     MUSIC
    [V, l] = eig(double(H));
    nelem = 1;
        
    rightMat= V(:, 1:end-nelem);
    
    cols = size(rightMat,2);
    P = randperm(cols);
    B = rightMat(:,P);
    
    resultMat = e_term_exp*B;
    
%     betaArrayBig = sum(abs((resultMat)).^2, 2);
    betaArrayBig = sum(abs(resultMat),2);
%     betaProfile = 1./betaArrayBig;
    betaProfile = betaArrayBig;
    betaProfile = reshape(betaProfile,[360,90]).';
    betaProfile = betaProfile ./ sum(sum(betaProfile));
    
%     Bartlett
%     rightMat= H{:,:};
%     resultMat = e_term_exp*rightMat;
%     
%     betaArrayBig = sum(abs((resultMat)).^2, 2);
%     betaProfile = betaArrayBig;
%     betaProfile = reshape(betaProfile,[360,90]).';
%     betaProfile = betaProfile ./ sum(sum(betaProfile));
    
    nbeta = 360;
    ngamma = 90;
    phi_max = 180;
    theta_max = 90;
    beta_min = -phi_max*(pi/180);
    beta_max = phi_max*(pi/180);
    gamma_min = 0*(pi/180);
    gamma_max = theta_max*(pi/180);      
    betaList = linspace(beta_min, beta_max, nbeta).';
    gammaList = linspace(gamma_min, gamma_max, ngamma);
    figure(12233);
    subplot(2,1,1)
    surf(betaList*180/pi, gammaList*180/pi, betaProfile, 'EdgeColor', 'none');
    set(gcf,'Renderer','Zbuffer')            
    xlabel('Azimuth (Degree)');
    ylabel('Elevation (Degree)');
    title(sprintf('AOA profile (side view)'));         
    subplot(2,1,2)
    surf(betaList*180/pi, gammaList*180/pi, betaProfile, 'EdgeColor', 'none');
    set(gcf,'Renderer','Zbuffer');
    view(2)
    title('AOA profile Top View');
    xlabel('Beta');
    ylabel('Gamma');

end