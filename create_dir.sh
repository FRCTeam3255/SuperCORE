#!/bin/bash
for DIR in $(find ./releases -type d); do
  (
    echo -e "<body>
	<h1>SuperCORE Maven Repo</h1>
	<h2><a href=\"/\">Home</a>
		<a href=\"/releases/com/frcteam3255/supercore/\">Releases</a>
		<a href=\"https://github.com/FRCTeam3255/SuperCORE\">Source</a>
	</h2>
	<hr />
	<pre>"
    ls -1pa "${DIR}" | grep -v "^\./$" | grep -v "index.html" | awk '{ printf "<a href=\"%s\">%s</a>\n",$1,$1 }' 
    echo -e "</pre>\n</body>\n</html>"
    echo -e "<footer>
	<hr />
	<pre>
Created by <a href=\"https://supernurds.com\" target=\"_blank\">FRC Team 3255 - SuperNURDs</a>
	</pre>
</footer>"
  ) > "${DIR}/index.html"
done
