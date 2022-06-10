<html>
<head>

<style>
.result-xy {
    height: 400px; width: 600px;
    max-width: calc(100% - 40px);
    margin: 80px 40px 80px 40px;
    font-size: 0;
    border-top: 1px solid #E0E2E6;  border-bottom: 1px solid #E0E2E6;
    background-image: url("Maze.png");
    background-size: 600px 400px;
    position: relative;
    text-align: center;
}
.result-xy > s {
    position: absolute;
    width: 12px; height: 12px;
    background-color: rgb(6, 6, 240); /*Maze setting for dot */
    color: #f0e10d;
    border: 2px solid;
    border-radius: 50%;
    margin: auto;
}
.result-xy i[line] {
    position: absolute;
    box-sizing: border-box;
    border-left: 0px solid #fff; border-right: 0px solid #fff;
    height: 4px;
    background-color: #f3540b7e;
    transform-origin: left center;
    left: 50%; top: 50%; margin-top: -1px;
    -ms-pointer-events: none;
    pointer-events: none;
    z-index: 1;
}

.result-z {
    height: 400px; width: 700px;
    max-width: calc(100% - 40px);
    margin: 80px 40px 80px 40px;
    font-size: 0;
    border-top: 1px solid #E0E2E6;  border-bottom: 1px solid #E0E2E6;
    background: linear-gradient(to top, #E0E2E6, #E0E2E6 1px, transparent 1px);
    background-size: 100% 50px;
    position: relative;
    text-align: center;
}
.result-z::after {
    addcolumn:'accel';
    content:'400\A 300\A 200\A 100\A 0\A -100\A -200\A -300 \A -400';
    white-space: pre-wrap;
    position: absolute;
    font-size: 14px;
    line-height: 50px;
    top: -26px; left: -1.5em;
}
.yaxis {
    content:'accel';
    white-space: pre-wrap;
    position: absolute;
    font-size: 14px;
    line-height: 50px;
    top: -26px; left: -1.5em;
}
.result-z-y {
    display: inline-block;
    width: 1px; height: 100%;
    background-color: #eee;
    position: absolute;
    /*margin-left: 0px; */
}
/*
.result-z-y + .result-z-y {
    margin-left: 39px;
}
 */
.result-z-y::after {
    content: attr(data-time)'ms';
    position: absolute;
    bottom: -2em;
    left: 0; right: 0;
    text-align: center;    
    font-size: 12px;
    color: gray;
}
.result-z > s {
    position: absolute;
    width: 6px; height: 6px;
    background-color: #fff; /*Chart setting for dot */
    color: #0c10f1;
    border: 3px solid; 
    border-radius: 50%;
    top: 15px; left: 0; right: 0;
    margin: 0;
}
.result-z i[line] {
    position: absolute;
    box-sizing: border-box;
    border-left: 3px solid #fff; border-right: 3px solid #fff;
    height: 3px;
    background-color: #0c10f1;/*Chart setting for line */
    transform-origin: left center;
    left: 50%; top: 50%; margin-top: -1px;
    -ms-pointer-events: none;
    pointer-events: none;
    z-index: 1;
}
</style>
<meta http-equiv="refresh" content="1000">
</head>

<body>
<table>
    <tr> <!-- title setting  -->
        <td align="center">Maze Tracing...</td>
        <td align="center">Tremor...</td>
    </tr>
    <tr><!-- sessionId setting  -->
        <?php
            $sessionId = 1;
            if ($_POST["sessionId"] != null) {
                $sessionId = $_POST["sessionId"];
            }
        ?>
        <td align="center"> <!-- MAZE button-->
            <form action="Final_display.php" method="post">
                SessionId: <input type="text" id="sessionId" value="<?php echo $sessionId; ?>" name="sessionId">
                <input type="submit" value="go">
                <input type="submit" value="next" onClick="document.getElementById('sessionId').value++">
                <input type="submit" value="previous" onClick="document.getElementById('sessionId').value--">
                
            </form>
        </td> 
        
        <?php
            $page = 1;
            if ($_POST["curPage"] != null) {
                $page = $_POST["curPage"];
            }
        ?>

        <td align="center"><!-- CHART button setting -->
            
            <form action="Final_display.php" method="post">
                SessionId: <input type="text" id="sessionId" value="<?php echo $sessionId; ?>" name="sessionId">
                Current Page: <input type="text" id="curPage" value="<?php echo $page; ?>" name="curPage">
                <input type="submit" value="go">
                <input type="submit" value="next" onClick="document.getElementById('curPage').value++">
                <input type="submit" value="previous" onClick="document.getElementById('curPage').value--">
                
            </form>
        </td>
        
    </tr>
    <tr>
        <td>
            <div id="chartXY" class="result-xy"><!-- MAZE data -->
                <?php

                $servername = "localhost";
                $dbname = "Puzzle";
                $username = "root";
                $password = "raspberry";
                
                // Create connection
                $conn = new mysqli($servername, $username, $password, $dbname);
                // Check connection
                if ($conn->connect_error) {
                    die("Connection failed: " . $conn->connect_error);
                } 
                
                $sql = "SELECT ID, SessionID, TimeStamp, X_value, Y_value FROM MPos WHERE SessionID=" . $sessionId . " ORDER BY id";
                $x_max = 11.748;//distance to origin from X axis
                $y_max = 3.625;
                $x_max_pic = 535;//html table X axis distance
                $y_max_pic = 315;
                $x_offset = 0;
                $y_offset = 40;
                if ($result = $conn->query($sql)) {
                    while ($row = $result->fetch_assoc()) {
                        $row_ID = $row["ID"];
                        $row_SessionID = $row["SessionID"];
                        $row_TimeStamp = $row["TimeStamp"];
                        $row_X_value = $row["X_value"]; 
                        $row_Y_value = $row["Y_value"];
                        //coordinate calculation
                        $x = $x_offset + ($row_X_value / $x_max) * $x_max_pic;
                        $y = $y_offset + $y_max_pic - ($y_max - $row_Y_value) * $y_max_pic / ($y_max * 2);

                        //display coordinate
                        echo '<s title="id:' . $row_ID . ', x:' . $x . ', y:' . $y .'" style="position: absolute;left: ' . $x . 'px;top: ' . $y . 'px;"></s>';
                    }
                
                    $result->free();
                }
                
                $conn->close();
                
                ?>

            </div>
        </td>
        <td>
            <div style="border:1px solid black;width:700px;height:500px;overflow-y:scroll;overflow-x:scroll;"><!-- adding scrolling on CHART -->  
                <p style="width:350%;">
                   

                    <div id="chartZ" class="result-z"> <!-- CHART data -->   
                        <?php

                                $servername = "localhost";
                                $dbname = "Puzzle";
                                $username = "root";
                                $password = "raspberry";
                                
                                // Create connection
                                $conn = new mysqli($servername, $username, $password, $dbname);
                                // Check connection
                                if ($conn->connect_error) {
                                    die("Connection failed: " . $conn->connect_error);
                                } 
                                
                                $pageSize = 1000;
                                $offset = $pageSize * ($page - 1);
                                
                                //This one only shows every 100ms'th data entry.  Display times are much quicker! (David)
                                //$sql = "SELECT ID, SessionID, TimeStamp, Filtered_Vector, Hertz FROM Accel WHERE SessionID=" . $sessionId . " AND TimeStamp%100=0 ORDER BY id LIMIT " . $offset . "," . $pageSize;
                                
                                //The original line Lin/Nic had
                                $sql = "SELECT ID, SessionID, TimeStamp, Filtered_Vector, Hertz FROM Accel WHERE SessionID=" . $sessionId . " ORDER BY id LIMIT " . $offset . "," . $pageSize;
                                $interval = 50;
                                $y_max = 300;
                                $y_max_pic = 400;
                                $index = 0;
                                if ($result = $conn->query($sql)) {
                                    while ($row = $result->fetch_assoc()) {
                                        $row_ID = $row["ID"];
                                        $row_SessionID = $row["SessionID"];
                                        $row_TimeStamp = $row["TimeStamp"]; 
                                        $row_Filtered_Vector = $row["Filtered_Vector"];
                                        $row_Hertz = $row["Hertz"];
                                        

                                        $x_display = $index * $interval;
                                        $x = $index * $interval - 5;
                                        $x_txt = $x - 7;
                                        //$y = $y_max_pic - (($row_Filtered_Vector / $y_max) * $y_max_pic)-$y_max_pic/2;
                                        $y = $y_max_pic / 2 - $row_Filtered_Vector / 2 - 4;
                                        $y_txt = $y - 20;
                                        //$h = $row_Hertz * 2;
                                        $h = 6;

                                        echo '<div class="result-z-y" data-time="' . $row_TimeStamp . '" style="left: ' .$x_display . 'px;"></div>'; //x axis, showing interval
                                        echo '<div style="position:absolute;left: ' . $x_txt . 'px;top: ' . $y_txt . 'px;width:20px;height:20px;font-size: 10;">' . $row_Hertz . 'Hz</div>'; //axis location
                                        echo '<s title="id:' . $row_ID . ', x:' . $row_TimeStamp . 'ms, y:' . $y . 'px, z:' . $row_Filtered_Vector . ', h:' . $row_Hertz . '" style="left: ' . $x . 'px;top: ' . $y . 'px;height: ' . $h . '"></s>';//vector and frequency
                                        $index ++;
                                    }
                                
                                    $result->free();
                                }
                                
                                $conn->close();
                                
                        ?>
                    </div>
                    
                </p>
            </div>
        </td>
    </tr>
</table>

<script>
    var eleDotXYs = document.querySelectorAll('#chartXY s');
    var eleDotZs = document.querySelectorAll('#chartZ s');

    // Line plotting
    var fnLineChart = function (eles) {
        [].slice.call(eles).forEach(function (ele, index) {
            var eleNext = eles[index + 1];
            if (!eleNext) { return;  }
            var eleLine = ele.querySelector('i');
            if (!eleLine) {
                eleLine = document.createElement('i');
                eleLine.setAttribute('line', '');
                ele.appendChild(eleLine);
            }
            // get coordinate
            var boundThis = ele.getBoundingClientRect();
            // next coordinate
            var boundNext = eleNext.getBoundingClientRect();
            // 
            var x1 = boundThis.left, y1 = boundThis.top;
            var x2 = boundNext.left, y2 = boundNext.top;
            // line distance
            var distance = Math.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
            // line direction
            var radius = Math.atan((y2 - y1) / (x2 - x1));
            // line style
            eleLine.style.width = distance + 'px';
            eleLine.style.msTransform = 'rotate('+ radius +'rad)';
            eleLine.style.transform = 'rotate('+ radius +'rad)';
        });
    };

    // call dot/line plotting
    //fnLineChart(eleDotXYs);    //MAZE line plotting(disabled)
    fnLineChart(eleDotZs);    //CHART line plotting

    // resize when monitor size changes
    window.addEventListener('resize', function () {
        fnLineChart(eleDotXYs);
        fnLineChart(eleDotZs);
    });
</script>
</body>
</html>
