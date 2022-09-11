#!/bin/bash
# Creates web directory for releases
for DIR in $(find ./releases -type d); do
if [[ "$DIR" != *"javadoc"* ]]; then
  (
    echo "<body>
	<h1>SuperCORE Maven Repo</h1>
	<h2><a href=\"/SuperCORE/\">Home</a>
		<a href=\"/SuperCORE/releases/com/frcteam3255/supercore/\">Releases</a>
		<a href=\"https://github.com/FRCTeam3255/SuperCORE\">Source</a>
		<a href=\"/SuperCORE/releases/com/frcteam3255/supercore/javadoc-latest\">JavaDocs-latest</a>
	</h2>
	<hr />
	<pre>"
    ls -1pa "${DIR}" | grep -v "^\./$" | grep -v "index.html" | awk '{ printf "<a href=\"%s\">%s</a>\n",$1,$1 }' 
    echo "</pre>"
    echo "</body>"
	echo "</html>"
    echo "<footer>
	<hr />
	<pre>
Created by <a href=\"https://supernurds.com\" target=\"_blank\">FRC Team 3255 - SuperNURDs</a>
	</pre>
</footer>"
  ) > "${DIR}/index.html"
  fi
done
