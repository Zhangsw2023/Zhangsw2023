function [s]=ComputeArcLengthOfBsplinesq(row,X, Y,Z,U,a,b)
%���ݲ�����B�������߲��������㻡��
% if a~=0
%     a = (a*1.0e8 - 1.0) / 1.0e8;
% end
% if b~=0
%     b = (a*1.0e8 - 1.0) / 1.0e8;
% end
     h = (b - a) / 4;
     x0 = a;
	 x1 = a + h;
	 x2 = x1 + h;
	 x3 = x2 + h;
	x4 = b;
    f0 = ComTanVecBspline(row, X, Y, Z, U, x0);
	f1 = ComTanVecBspline(row, X, Y, Z, U, x1);
	f2 = ComTanVecBspline(row, X, Y, Z, U, x2);
	f3 = ComTanVecBspline(row, X, Y, Z, U, x3);
	f4 = ComTanVecBspline(row, X, Y, Z, U, x4);

	s = 2 * h*(7 * f0 + 32 * f1 + 12 * f2 + 32 * f3 + 7 * f4) / 45;
end
