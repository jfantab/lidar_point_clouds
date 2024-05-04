function alpha = computeAlpha(tx, tz, rz)

alpha = atan2(tz, tx) - sign(tz) * rz;

end
