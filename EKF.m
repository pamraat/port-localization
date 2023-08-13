function [munew, sigmanew] = EKF(mu, sigma, u, z, R, Q, g, G, h, H)
    muPred = g(mu, u);
    GJac = G(mu, u);
    if all(u == 0) R = zeros(size(R)); end
    sigmaPred = GJac*sigma*GJac' + R;
    zPred = h(muPred); zPred(isnan(zPred)) = 0; zPred(z==0) = 0;
    HJac = H(muPred); HJac(isnan(HJac)) = 0;
    if (norm(z - zPred) > 1 && length(z)>3) HJac = zeros(size(HJac)); end
    K = sigmaPred*HJac'/(HJac*sigmaPred*HJac' + Q);
    munew = muPred + K*(z - zPred);
    sigmanew = (eye(size(sigma, 1)) - K*HJac)*sigmaPred;
end