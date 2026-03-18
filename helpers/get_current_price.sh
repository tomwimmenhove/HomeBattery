#/bin/bash

./helpers/get_prices.sh | jq '
  .data[]
  | (.date | sub("\\+00:00$"; "Z") | fromdateiso8601) as $t
  | select(now >= $t and now < ($t + 3600))
'

