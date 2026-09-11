function separation = los_separation_deg(az, el, ref_az, ref_el)
%LOS_SEPARATION_DEG Great-circle angle between azimuth/elevation directions.
u = [cosd(el(:).').*sind(az(:).'); ...
    cosd(el(:).').*cosd(az(:).'); sind(el(:).')];
v = [cosd(ref_el(:).').*sind(ref_az(:).'); ...
    cosd(ref_el(:).').*cosd(ref_az(:).'); sind(ref_el(:).')];
product = cross(u, v, 1);
separation = reshape(atan2d(sqrt(sum(product.^2, 1)), sum(u.*v, 1)), size(az));
end
