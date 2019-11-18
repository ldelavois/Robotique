var api = null;

$(document).ready(function() {
    $('.slider').simpleSlider({
        'range': [-180, 180]
    });

    $('.slider-xyz').simpleSlider({
        'range': [-0.35, 0.35]
    });
    
    new QWebChannel(qt.webChannelTransport, function(channel) {
        api = channel.objects.api;

        setInterval(function() {
            api.debugInfo(function(info) {
                $('.debug').html(info);
            });
        }, 40);

        function updateAngles() {
            api.setAngles(
                $('input[name=angle1]').val(),
                $('input[name=angle2]').val(),
                $('input[name=angle3]').val()
            );
        }

        updateAngles();
    
        $('.slider').change(function() {
            updateAngles();
        });

        function updateXYZs() {
            api.setXYZ(
                $('input[name=x]').val(),
                $('input[name=y]').val(),
                $('input[name=z]').val()
            );
        }

        updateXYZs();
    
        $('.slider-xyz').change(function() {
            updateXYZs();
        });

        function setMode(mode) {
            if (mode == 'angle') {
                $('.mode-angle').show();
                $('.mode-xyz').hide();
            } else if (mode == 'xyz' || mode == 'xyz-iterative') {
                $('.mode-angle').hide();
                $('.mode-xyz').show();
            } else {
                $('.mode-angle').hide();
                $('.mode-xyz').hide();
            }

            api.setMode(mode);
        }
    
        setMode($('.mode').val());
        $('.mode').change(function() {
            setMode($(this).val());
        });
    });
});