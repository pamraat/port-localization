function [Xnew, w] = PF(X, u, z, R, Q, g, h)
    for i = 1:size(X, 2)
        XPredchk(:, i) = g(X(:, i), u);
        XPred(:, i) = g(X(:, i), u) + mvnrnd(zeros(3, 1), R)';
        zPred(:, i) = h(XPred(:, i)); zPred(isnan(zPred)) = 0;
        w(1, i) = mvnpdf(zPred(:, i), z, Q);
    end
    if all(w == 0, 'all')
        Xnew = XPred;
        return
    end
    w = w/sum(w);
    Xnew = XPred(:, randsample(size(X, 2), size(X, 2), true, w));
end